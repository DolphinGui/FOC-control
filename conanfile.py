# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain
from conan.tools.env import VirtualBuildEnv

required_conan_version = ">=2.2.2"


class demos(ConanFile):
    python_requires = "libhal-bootstrap/[^4.0.0]"
    python_requires_extend = "libhal-bootstrap.demo"

    options = {
        "platform": ["ANY"],
        "variant": [None, "ANY"],
        "board": [None, "ANY"]
    }

    default_options = {
        "platform": "ANY",
        "variant": None,
        "board": None,
    }


    def configure(self):
        # self.options["mp-units/*"].freestanding = True
        self.options["mp-units/*"].contracts = "none"
        self.options["mp-units/*"].std_format = True


    def generate(self):
        virt = VirtualBuildEnv(self)
        virt.generate()
        tc = CMakeToolchain(self)
        if str(self.options.platform).startswith("rp2"):
            if self.options.board:
                tc.cache_variables["PICO_BOARD"] = str(self.options.board)
        tc.generate()
        cmake = CMakeDeps(self)
        cmake.generate()

    def requirements(self):
        self.requires("libhal-util/[^5.4.0]")
        self.requires("libhal-arm-mcu/latest")
        # bootstrap = self.python_requires["libhal-bootstrap"]
        # bootstrap.module.add_demo_requirements(self)
        self.requires("mp-units/2.4.0")
        if str(self.options.platform).startswith("rp2"):
          self.tool_requires("picotool/2.2.0")
