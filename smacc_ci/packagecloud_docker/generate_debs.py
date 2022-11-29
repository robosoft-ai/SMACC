# Copyright 2021 RobosoftAI Inc.
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

# /bin/python
# Author: Pablo Inigo Blasco (github: pabloinigoblasco) 2019


import rospkg
import subprocess
import os
import shutil
import re


def build_deb_package(
    workspace_source_folder, package_name, packagepath, ubuntu_version, ros_distro, already_visited
):
    os.chdir(workspace_source_folder)
    print("------------------------------------------------------------")
    print("Working folder: " + str(os.getcwd()))
    print("Building debian package: " + str(package_name))
    cmd = (
        "bloom-generate rosdebian --os-name ubuntu --os-version "
        + str(ubuntu_version)
        + " --ros-distro "
        + str(ros_distro)
        + " "
        + str(packagepath)
    )
    print(cmd)
    bloomprocess = subprocess.Popen(cmd, shell=True)
    bloomprocess.wait()

    # finding actual source package directory of the package in the workspace
    develpackagefolder = None
    for root, dirs, files in os.walk(workspace_source_folder, topdown=False):
        if "package.xml" in files and package_name == root.split("/")[-1] and "debian" not in root:
            print(root)
            develpackagefolder = root
            break

    if develpackagefolder is None:
        print("ERROR: package " + str(package_name) + " not found in workspace")

    localpackagepath = develpackagefolder
    print("local package path: {localpackagepath} ".format(**locals()))

    # cleaning possible previous wrong builds
    if os.path.exists(os.path.join(localpackagepath, "debian")):
        shutil.rmtree(os.path.join(localpackagepath, "debian"))

    # executing bloom to generate debian-fakeroot build data
    print(cmd)
    bloomprocess = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    bloomprocess.wait()
    print("bloom process finished: " + str(bloomprocess.returncode))

    for line in bloomprocess.stdout:
        print(str(line))

    print("------------------------------------------------------------")
    os.chdir(workspace_source_folder)
    print(" - Local package path: " + localpackagepath)
    print(" - Workspace directory:")
    print(os.getcwd())
    print(" - list localpackagepath:")
    print(os.listdir(localpackagepath))

    # copy the debian folder generated by bloom inside the package
    shutil.move("debian", os.path.join(localpackagepath, "debian"))

    os.chdir(localpackagepath)
    fakerootprocess = subprocess.Popen("fakeroot debian/rules binary", shell=True)
    (result, err) = fakerootprocess.communicate()

    print("--------------------")
    os.chdir(workspace_source_folder)

    firstregexstr = ".*ros-" + ros_distro + r"-.*\.deb$"
    regexstr = ".*ros-" + ros_distro + "-" + package_name.replace("_", "-") + r"_.*\.deb$"
    print("Finding deb package: " + str(regexstr))

    thisfolderfiles = []
    for root, dirs, files in os.walk(workspace_source_folder, topdown=False):
        thisfolderfiles = thisfolderfiles + [os.path.join(root, f) for f in files]

    debianfiles = [f for f in thisfolderfiles if re.search(firstregexstr, f)]
    print("DETECTED DEBIAN FILES:")
    for d in debianfiles:
        print("- {d}".format(**locals()))

    visited_debian_files = [
        f for f in debianfiles if re.search(regexstr, f) and f not in already_visited
    ]
    debianfilename = visited_debian_files[0]
    print("VISITED DEBIAN FILES:")
    for d in visited_debian_files:
        print("- {d}".format(**locals()))

    print("Debian file found: ")
    print(debianfilename)

    os.chdir(localpackagepath)
    try:
        shutil.rmtree(".obj-x86_64-linux-gnu")
    except Exception as ex:
        print("no .obj-x86_64-linux-gnu folder found: " + str(ex))

    try:
        shutil.rmtree("debian")
    except Exception as ex:
        print("no debian folder found" + str(ex))

    print("installing debian package (required to build new debs): " + debianfilename)
    installdebiantask = subprocess.Popen("sudo dpkg -i " + debianfilename, shell=True)
    installdebiantask.wait()

    return debianfilename


def iterate_debian_generation(
    workspace_source_folder, package_names, identified_install_packages, osversion, rosversion
):
    os.chdir(workspace_source_folder)
    debianfiles = []
    for pname in package_names:
        try:
            debianfiles.append(
                build_deb_package(
                    workspace_source_folder,
                    pname,
                    identified_install_packages[pname],
                    osversion,
                    rosversion,
                    debianfiles,
                )
            )
        except Exception as ex:
            print(ex)
            print("ERROR: package " + str(pname) + " could not be built")
            print("existing packages: " + str(identified_install_packages.keys()))
            raise ex

    return debianfiles


def get_identified_packages(workspace_folder):
    print("finding packages in workspace folder:" + workspace_folder)

    rospack = rospkg.RosPack()
    packages = rospack.list()
    packagesl = list(packages)
    identified_install_packages = {}

    print("packages2: " + str(packagesl))
    exclude_with_words = ["ridgeback", "mecanum", "catkin"]
    for pname in packagesl:
        packpath = rospack.get_path(pname)
        print(pname)
        if workspace_folder in packpath:
            if any([True for excludedword in exclude_with_words if excludedword in pname]):
                continue

            identified_install_packages[pname] = packpath
    return identified_install_packages


def push_debian_files_package_cloud(repo_owner, reponame, osname, osversion, debianfiles):
    for debf in debianfiles:
        print("pushing debfile")
        cmd = (
            "package_cloud push "
            + repo_owner
            + "/"
            + reponame
            + "/"
            + osname
            + "/"
            + osversion
            + " "
            + debf
        )
        print(cmd)
        push_debian_task = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

        push_debian_task.wait()

        for line in push_debian_task.stdout:
            print(str(line))


def remove_debian_files(repo_owner, reponame, osname, osversion, debianfiles):
    for debf in debianfiles:
        shortdebfile = debf.split("/")[-1]
        print("removing/yanking debfile")
        push_debian_task = subprocess.Popen(
            "package_cloud yank "
            + repo_owner
            + "/"
            + reponame
            + "/"
            + osname
            + "/"
            + osversion
            + " "
            + shortdebfile,
            shell=True,
        )
        push_debian_task.wait()


# ------------------------ SMACC PACKAGES -----------------------


def create_and_push_smacc_debians(osname, osversion, rosversion, reponame, action, packages):
    workspace_source_folder = os.path.join(workspace_folder, relative_smacc_folder)
    identified_install_packages = get_identified_packages(workspace_folder)

    print("workspace source folder: " + workspace_source_folder)
    print("packages: " + str(packages))
    print("--")
    print("identified packages: " + str(identified_install_packages.keys()))

    if "build" in action:
        smacc_debian_files = iterate_debian_generation(
            workspace_source_folder, packages, identified_install_packages, osversion, rosversion
        )
    else:
        smacc_debian_files = []
        for package_name in packages:
            firstregexstr = ".*ros-" + rosversion + r"-.*\.deb$"
            regexstr = ".*ros-" + rosversion + "-" + package_name.replace("_", "-") + r"_.*\.deb$"
            print("Finding deb package: " + str(regexstr))

            thisfolderfiles = []
            for root, dirs, files in os.walk(workspace_source_folder, topdown=False):
                thisfolderfiles = thisfolderfiles + [os.path.join(root, f) for f in files]

            debianfiles = [f for f in thisfolderfiles if re.search(firstregexstr, f)]
            print("DETECTED DEBIAN FILES:")
            for d in debianfiles:
                print("- {d}".format(**locals()))
                smacc_debian_files.append(d)

    if "push" in action:
        if reponame is None:
            print("ERROR: reponame is required for push action")
            raise Exception("reponame is required for push action")

        create_repo_task = subprocess.Popen("package_cloud repository create smacc", shell=True)
        create_repo_task.wait()

        # ----- PUSHING TO SMACC --------------
        remove_debian_files(repo_owner, "smacc", osname, osversion, smacc_debian_files)
        push_debian_files_package_cloud(
            repo_owner, reponame, osname, osversion, smacc_debian_files
        )

    return smacc_debian_files


if __name__ == "__main__":
    # === requirements for the build machine ==
    # sudo apt-get install ruby-dev rake
    # sudo gem update --system
    # sudo gem install package_cloud
    # == OR ==
    # curl -s https://packagecloud.io/install/repositories/fdio/tutorial/script.deb.sh | sudo bash

    import argparse
    import argcomplete

    rospack = rospkg.RosPack()
    packages = rospack.list()
    packagesl = list(packages)

    parser = argparse.ArgumentParser()

    parser.add_argument("-src_folder", help="smacc workspace folder", required=True)

    parser.add_argument(
        "-ros_version", help="The version of ros, ie: noetic", default="noetic", type=str
    )

    parser.add_argument(
        "-ubuntu_version", help="The version of ros, ie: focal", default="focal", type=str
    )

    parser.add_argument("-repo_owner", help="Package cloud repo owner", required=True)
    parser.add_argument("-repo_name", help="Package cloud repo name", default=None)
    parser.add_argument("-token", help="Package cloud repo token", default="")

    parser.add_argument("-packages", help="packages", nargs="+", required=True)

    parser.add_argument("-action", default="build_push", help="build|push|build_push")

    parser.add_argument("-help", help="Help command")

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    osname = "ubuntu"
    osversion = args.ubuntu_version
    ros_version = args.ros_version

    print("args:" + str(args))
    print("rosversion: " + ros_version)
    print("ubuntu: " + osversion)

    relative_smacc_folder = args.src_folder
    workspace_folder = os.path.abspath(os.path.join(os.getcwd(), "."))

    repo_owner = args.repo_owner

    print("CREATING TOKEN FILE FOR PACKAGE CLOUD:")
    homefolder = "/tmp"  # os.getenv("HOME")
    packagecloud_token_filepath = os.path.join(homefolder, ".packagecloud")

    outfile = open(packagecloud_token_filepath, "w")
    outfile.write('{"token":"%s"}' % args.token)
    outfile.close()

    smacc_debians = create_and_push_smacc_debians(
        osname, osversion, ros_version, args.repo_name, args.action, args.packages
    )
    print("SMACC DEBIAN'S: " + str(smacc_debians))
