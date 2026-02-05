from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps


class foc_dashRecipe(ConanFile):
    name = "foc_dash"
    version = "0.1"
    package_type = "application"

    # Optional metadata
    license = "MIT"
    author = "<Shin Umeda> <umeda.shin@gmail.com>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "A dashboard for monitoring and tuning BLDC controllers"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    settings = "os", "compiler", "build_type", "arch"

    exports_sources = "CMakeLists.txt", "src/*"

    requires = (
        "glfw/[>=3.4]",
        "argparse/[>=3.2]",
        "asio/[>=1.36]",
        "fmt/[>=12.1.0]",
    )

    def configure(self):
      pass
        #if self.settings.os == "Linux":
        #    self.options["glfw"].with_wayland = True
        #    self.options["glfw"].with_x11 = False
        #    self.options["xkbcommon"].with_wayland = True
        #    self.options["xkbcommon"].with_x11 = False

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
