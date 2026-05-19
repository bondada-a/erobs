"""Unit tests for VLMDetector — mocks the HTTP layer."""
from __future__ import annotations

import io
import os
import tempfile
from unittest.mock import MagicMock, patch

import pytest
from PIL import Image

# Skip the entire module if httpx isn't installed in the ROS env.
httpx = pytest.importorskip("httpx")

from beambot.detection.vlm_detector import VLMDetector, VLMDetectorError  # noqa: E402


def _tmp_png(width=320, height=240) -> str:
    img = Image.new("RGB", (width, height), (200, 100, 50))
    fd, path = tempfile.mkstemp(suffix=".png")
    os.close(fd)
    img.save(path, format="PNG")
    return path


def _mock_response(status_code: int, json_body: dict | None = None, text: str = ""):
    resp = MagicMock(spec=httpx.Response)
    resp.status_code = status_code
    resp.text = text or (str(json_body) if json_body else "")
    if json_body is not None:
        resp.json = MagicMock(return_value=json_body)
    else:
        resp.json = MagicMock(side_effect=ValueError("no json"))
    return resp


@pytest.fixture
def detector():
    return VLMDetector(service_url="http://localhost:8765", timeout_seconds=2.0)


def test_detect_success(detector):
    img = _tmp_png()
    fake = _mock_response(200, {
        "x": 123.4, "y": 56.7, "confidence": 0.9,
        "backend": "stub", "raw": {"foo": "bar"},
    })
    with patch("beambot.detection.vlm_detector.httpx.post", return_value=fake) as p:
        out = detector.detect(img, "Point to the sample.")
    assert out["pixel_x"] == 123.4
    assert out["pixel_y"] == 56.7
    assert out["confidence"] == 0.9
    assert out["backend"] == "stub"
    assert out["raw"] == {"foo": "bar"}
    p.assert_called_once()
    # Verify base64 image was sent.
    sent = p.call_args.kwargs["json"]
    assert "image_b64" in sent
    assert sent["prompt"] == "Point to the sample."


def test_detect_missing_file(detector):
    with pytest.raises(FileNotFoundError):
        detector.detect("/nonexistent/image.png", "x")


def test_detect_with_image_bytes_skips_file_check(detector):
    img = Image.new("RGB", (100, 100))
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    fake = _mock_response(200, {
        "x": 1, "y": 2, "confidence": 0.5, "backend": "stub", "raw": {},
    })
    with patch("beambot.detection.vlm_detector.httpx.post", return_value=fake):
        out = detector.detect("ignored", "x", image_bytes=buf.getvalue())
    assert out["pixel_x"] == 1.0


def test_detect_http_error_status(detector):
    img = _tmp_png()
    fake = _mock_response(500, text="internal error")
    with patch("beambot.detection.vlm_detector.httpx.post", return_value=fake):
        with pytest.raises(VLMDetectorError, match="HTTP 500"):
            detector.detect(img, "x")


def test_detect_timeout(detector):
    img = _tmp_png()
    with patch(
        "beambot.detection.vlm_detector.httpx.post",
        side_effect=httpx.TimeoutException("boom"),
    ):
        with pytest.raises(VLMDetectorError, match="timed out"):
            detector.detect(img, "x")


def test_detect_network_error(detector):
    img = _tmp_png()
    with patch(
        "beambot.detection.vlm_detector.httpx.post",
        side_effect=httpx.ConnectError("no route"),
    ):
        with pytest.raises(VLMDetectorError, match="request failed"):
            detector.detect(img, "x")


def test_detect_malformed_response_missing_field(detector):
    img = _tmp_png()
    fake = _mock_response(200, {"x": 1.0, "y": 2.0})  # missing confidence + backend
    with patch("beambot.detection.vlm_detector.httpx.post", return_value=fake):
        with pytest.raises(VLMDetectorError, match="missing required field"):
            detector.detect(img, "x")


def test_detect_non_json_response(detector):
    img = _tmp_png()
    fake = _mock_response(200, json_body=None, text="not json")
    with patch("beambot.detection.vlm_detector.httpx.post", return_value=fake):
        with pytest.raises(VLMDetectorError, match="not valid JSON"):
            detector.detect(img, "x")


def test_health(detector):
    fake = _mock_response(200, {"status": "ok", "backend": "stub", "loaded": True})
    fake.raise_for_status = MagicMock()
    with patch("beambot.detection.vlm_detector.httpx.get", return_value=fake):
        h = detector.health()
    assert h["status"] == "ok"
    assert h["loaded"] is True


def test_info(detector):
    fake = _mock_response(200, {"service": "vlm_pointing", "active": {"name": "stub"}})
    fake.raise_for_status = MagicMock()
    with patch("beambot.detection.vlm_detector.httpx.get", return_value=fake):
        i = detector.info()
    assert i["service"] == "vlm_pointing"


def test_health_failure(detector):
    fake = _mock_response(500, text="bad")
    fake.raise_for_status = MagicMock(
        side_effect=httpx.HTTPStatusError("500", request=MagicMock(), response=fake)
    )
    with patch("beambot.detection.vlm_detector.httpx.get", return_value=fake):
        with pytest.raises(VLMDetectorError, match="health check failed"):
            detector.health()


def test_service_url_strips_trailing_slash():
    d = VLMDetector(service_url="http://example.com:1234/")
    assert d.service_url == "http://example.com:1234"
