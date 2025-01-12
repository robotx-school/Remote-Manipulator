import pkg_resources
import os

install_loc = pkg_resources.get_distribution("urx").location
os.system(f"unzip -o urx_patched.zip -d {install_loc}")

