"""Run an ament test on a reserved, localhost-only ROS domain."""

import os
from itertools import cycle

import ament_cmake_test
import domain_coordinator


if __name__ == "__main__":
    inherited_domain = os.environ.get("ROS_DOMAIN_ID", "0")
    for name in (
        "ROS_DOMAIN_ID", "DISABLE_ROS_ISOLATION", "ROS_DISCOVERY_SERVER",
        "ROS_STATIC_PEERS", "ROS_LOCALHOST_ONLY", "ROS_SUPER_CLIENT",
        "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
        "CYCLONEDDS_URI", "RMW_FASTRTPS_USE_QOS_FROM_XML",
    ):
        os.environ.pop(name, None)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["SKIP_DEFAULT_XML"] = "1"
    domains = cycle(domain for domain in range(215, 230) if str(domain) != inherited_domain)
    with domain_coordinator.domain_id(selector=lambda: next(domains)) as domain:
        os.environ["ROS_DOMAIN_ID"] = str(domain)
        raise SystemExit(ament_cmake_test.main())
