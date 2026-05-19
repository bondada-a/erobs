"""Wire-format tests for the VLM service.

Uses the `stub` backend so no GPU / model weights are required. Verifies
the FastAPI surface that ROS-side clients will hit.
"""
from __future__ import annotations

import base64
import io

import pytest
from fastapi.testclient import TestClient
from PIL import Image

from vlm_service.server import create_app


def _png_b64(width: int = 640, height: int = 480, color=(128, 128, 128)) -> str:
    img = Image.new("RGB", (width, height), color)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


@pytest.fixture(scope="module")
def client():
    app = create_app(backend_name="stub", load_now=True)
    return TestClient(app)


def test_health(client):
    r = client.get("/health")
    assert r.status_code == 200
    body = r.json()
    assert body["status"] == "ok"
    assert body["backend"] == "stub"
    assert body["loaded"] is True


def test_info(client):
    r = client.get("/info")
    assert r.status_code == 200
    body = r.json()
    assert body["service"] == "vlm_pointing"
    assert "stub" in body["available_backends"]
    assert "molmoact" in body["available_backends"]
    assert "molmoact2" in body["available_backends"]
    assert "robobrain25" in body["available_backends"]
    assert "robopoint" in body["available_backends"]
    assert body["active"]["name"] == "stub"


def test_point_returns_pixel_coords(client):
    r = client.post("/point", json={
        "image_b64": _png_b64(640, 480),
        "prompt": "Point to the sample.",
    })
    assert r.status_code == 200, r.text
    body = r.json()
    # Stub returns center +/- 10%, so coords should fall inside image.
    assert 0 <= body["x"] <= 640
    assert 0 <= body["y"] <= 480
    assert body["confidence"] == pytest.approx(0.99)
    assert body["backend"] == "stub"
    assert body["raw"]["image_size"] == [640, 480]


def test_point_deterministic_for_same_prompt(client):
    payload = {"image_b64": _png_b64(800, 600), "prompt": "Point to the puck."}
    r1 = client.post("/point", json=payload).json()
    r2 = client.post("/point", json=payload).json()
    assert r1["x"] == r2["x"]
    assert r1["y"] == r2["y"]


def test_point_different_prompts_yield_different_coords(client):
    img = _png_b64(640, 480)
    a = client.post("/point", json={"image_b64": img, "prompt": "alpha"}).json()
    b = client.post("/point", json={"image_b64": img, "prompt": "beta"}).json()
    assert (a["x"], a["y"]) != (b["x"], b["y"])


def test_point_rejects_bad_base64(client):
    r = client.post("/point", json={
        "image_b64": "this is not base64!!!",
        "prompt": "Point to anything.",
    })
    assert r.status_code == 400
    assert "base64" in r.json()["detail"].lower()


def test_point_rejects_non_image_bytes(client):
    bad = base64.b64encode(b"not an image").decode("ascii")
    r = client.post("/point", json={"image_b64": bad, "prompt": "x"})
    assert r.status_code == 400


def test_point_missing_field(client):
    r = client.post("/point", json={"prompt": "x"})  # no image
    assert r.status_code == 422


def test_point_batch(client):
    img = _png_b64(320, 240)
    r = client.post("/point_batch", json={
        "items": [
            {"image_b64": img, "prompt": "first"},
            {"image_b64": img, "prompt": "second"},
        ],
    })
    assert r.status_code == 200
    body = r.json()
    assert len(body["results"]) == 2
    assert body["results"][0]["backend"] == "stub"


def test_molmo_point_parser():
    """Unit test the Molmo output parser without loading a model."""
    from vlm_service.backends.molmoact import _parse_molmo_point

    text = 'I see it. <point x="50.0" y="25.0" alt="sample">sample</point>'
    r = _parse_molmo_point(text, 800, 600)
    assert r is not None
    assert r.x == pytest.approx(400.0)  # 50% of 800
    assert r.y == pytest.approx(150.0)  # 25% of 600


def test_molmo_point_parser_multi():
    from vlm_service.backends.molmoact import _parse_molmo_point

    text = '<points x1="10" y1="20" x2="50" y2="60" alt="things">things</points>'
    r = _parse_molmo_point(text, 1000, 1000)
    assert r is not None
    assert r.x == pytest.approx(100.0)
    assert r.y == pytest.approx(200.0)


def test_molmo_point_parser_no_match():
    from vlm_service.backends.molmoact import _parse_molmo_point
    assert _parse_molmo_point("no point here", 100, 100) is None


def test_robobrain_point_parser_json_list():
    from vlm_service.backends.robobrain25 import _parse_robobrain_point

    r = _parse_robobrain_point("Here it is: [[324, 188], [410, 205]]")
    assert r is not None
    assert r.x == 324.0
    assert r.y == 188.0
    assert r.raw["all_points"] == [[324, 188], [410, 205]]


def test_robobrain_point_parser_single_pair():
    from vlm_service.backends.robobrain25 import _parse_robobrain_point

    r = _parse_robobrain_point("coords: [42.5, 99.1]")
    assert r is not None
    assert r.x == pytest.approx(42.5)
    assert r.y == pytest.approx(99.1)


def test_robobrain_point_parser_no_match():
    from vlm_service.backends.robobrain25 import _parse_robobrain_point
    assert _parse_robobrain_point("nothing parseable here") is None


def test_robopoint_parser_normalized_tuples():
    from vlm_service.backends.robopoint import _parse_robopoint_points

    text = "Sure, here are the points: [(0.5, 0.25), (0.6, 0.3)]"
    r = _parse_robopoint_points(text, 800, 600)
    assert r is not None
    assert r.x == pytest.approx(400.0)  # 0.5 * 800
    assert r.y == pytest.approx(150.0)  # 0.25 * 600
    assert r.raw["all_points_norm"] == [(0.5, 0.25), (0.6, 0.3)]


def test_robopoint_parser_clamps_out_of_range():
    from vlm_service.backends.robopoint import _parse_robopoint_points

    r = _parse_robopoint_points("[(1.2, -0.1)]", 100, 100)
    assert r is not None
    assert r.x == 100.0  # clamped to 1.0
    assert r.y == 0.0    # clamped to 0.0


def test_robopoint_parser_no_match():
    from vlm_service.backends.robopoint import _parse_robopoint_points
    assert _parse_robopoint_points("no points here", 100, 100) is None
