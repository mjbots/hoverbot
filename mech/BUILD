# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

package(default_visibility = ["//visibility:public"])

load("//base:module_main.bzl", "module_main")
load("@rules_pkg//:pkg.bzl", "pkg_tar")

filegroup(
    name = "web_control_assets",
    srcs = [
        "web_control_assets/mjbots.png",
        "web_control_assets/index.html",
        "web_control_assets/styles.css",
        "web_control_assets/js/app.js",
    ],
)

cc_library(
    name = "mech",
    srcs = [
        "camera_driver.cc",
        "mime_type.cc",
        "pi3hat_wrapper.cc",
        "hoverbot.cc",
        "hoverbot_control.cc",
        "system_info.cc",
        "web_server.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//base",
        "@boost//:filesystem",
        "@dart",
        "@com_github_mjbots_mjlib//mjlib/base:pid",
        "@com_github_mjbots_mjlib//mjlib/io:exclusive_command",
        "@com_github_mjbots_mjlib//mjlib/io:selector",
        "@com_github_mjbots_mjlib//mjlib/multiplex:asio_client",
        "@com_github_mjbots_mjlib//mjlib/multiplex:frame",
        "@com_github_mjbots_mjlib//mjlib/multiplex:register",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:videoio",
        "@sophus",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : [
            "@raspicam",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
    }),
    copts = [
        "-Wno-gnu-designator",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : ["-DCOM_GITHUB_MJBOTS_RASPBERRYPI"],
    }),
    data = [
        ":web_control_assets",
    ],
    features = [
        "dbg",
    ],
)

# cc_test(
#     name = "test",
#     srcs = ["test/" + x for x in [
#         "test_main.cc",
#     ]],
#     deps = [
#         ":mech",
#         "@boost//:test",
#     ],
# )

module_main(
    name = "hoverbot",
    cname = "mjmech::mech::Hoverbot",
    prefix = "mech",
    deps = [":mech"],
)

pkg_tar(
    name = "hoverbot_deploy",
    extension = "tar",
    include_runfiles = True,
    package_dir = "hoverbot",
    srcs = [
        ":hoverbot",
    ],
    files = {
        "//configs:hoverbot.ini": "configs/hoverbot.ini",
        "//configs:hoverbot.cfg": "configs/hoverbot.cfg",
        "//utils:config_servos.py": "config_servos.py",
        "//utils:performance_governor.sh": "performance_governor.sh",
        "//utils:hoverbot.cmd" : "hoverbot.cmd",
        "//utils:hoverbot_screen.conf" : "hoverbot_screen.conf",
        "//utils:hoverbot-start.sh" : "hoverbot-start.sh",
        "//utils:start-robot.sh": "start-robot.sh",
        "web_control_assets/mjbots.png" : "web_control_assets/mjbots.png",
        "web_control_assets/index.html" : "web_control_assets/index.html",
        "web_control_assets/styles.css" : "web_control_assets/styles.css",
        "web_control_assets/js/app.js" : "web_control_assets/js/app.js",
    },
)
