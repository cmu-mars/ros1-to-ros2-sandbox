# rviz-diff

Creates a diff between one version of rviz, the ROS visualization package,
and a different, older branch.

## Running

### Some Setup/Dependencies

You need to have Python version 3.5 or higher, at least, to run `make-diff.py`.
This script has only been tested using Python 3.6.5.

To start, in some directory, clone the [rviz repository](https://github.com/ros2/rviz/tree/ros2)
using git. Put the location of that in the `DIFF_REPO_LOC` environment
variable. (It should probably be somewhere that is not inside this folder,
or anywhere in the directory for the ros1-to-ros2-sandbox repo either,
for that matter.)

It is important that the directory that give by `echo $DIFF_REPO_LOC`
is a git repository, since the script in `make-diff.py` will use git to
retrieve the older files.

Inside the rviz repo you cloned, make sure you have the `ros2` branch. Now
git fetch a ros1 branch of your choosing, such as `kinetic-devel`, which
will be the branch the `ros2` code is compared to.

#### Install gitpython

Set up a virtualenv and do `pip install gitpython`, which is the module we
use to checkout different branches in the rviz git repo.

### Actually running the script

Run with `python3 make-diff.py`.

#### Notes about the output
The command line output will be a lot of file names, and at the end, it
will also tell you the files that are in the ros2 branch but not in the
ros1 branch, so they are all totally new files, supposedly.

The script also copies all of the files that have counterparts -- judging by
name -- into directories relative to the rviz-diff directory. The diff
between that file and the new ROS2 file is put into the same location. If the
path of the file in ROS1 version of the rviz directory was `/rviz/path/to/name-of-file.cpp`,
then path of the file within this directory will be `rviz-diff/path/to/name-of-file.cpp`,
and the diff will be `rviz-diff/path/to/name-of-file.cpp.html`.

Additionally, the script adds the absolute path of both files it is
comparing in the diff output, for convenience. This is included in a comment
at the top of each file's "side" in the diff.

## How it finds matches

Programmers generally don't change the name of a file when they move it,
the script finds two things:

- the files that have supposedly been added in ROS2 branch
- the files that have since been deleted that are in the ROS1 branch.

It then tries to match up these files by the name of the file itself,
disregarding its parent directories. It records matches, and then checks
to see if the files are exactly the same using the `filecmp` Python
module. If they aren't exactly the same, a diff is created and stored
in a file with a similar name as the ROS1 branch file.
