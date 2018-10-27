# Librarian repository
## How to deploy this repo ?
Clone this repo in your catkin workspace (probably ~/catkin\_ws/) using :
~~~~
cd ~/catkin\_ws
git clone https://gitlab.doc.ic.ac.uk/HCR/Robot.git
~~~~
Then, rename it in 'src' 
~~~~
mv Robot/ src/
~~~~
## What is in this repo ?
There are many branches, **one for every package**. Feel free to create more branches, but 
make sure that your package branch is mergeable (don't put your tests or experiment
in the branch with your package name to keep the master clean after a merge).

The master branch is protected.
To push to the master branch, you have to make a pull request (or merge request).

The **librarian_msgs** branch is protected too. It contains the package librarian_msgs.
Every messages exchanged between different nodes must be in the librarian_msgs
package to make the integration easier.

If you have to change a message file (*.msg) feel free to make a pull request.

Make sure to merge your working branch with the librarian_msgs branch frequently in order 
to work with the last version of messages template. To do so, in your banch :
~~~~
git checkout YOUR_BRANCH_NAME
git fetch
git merge librarian_msgs
~~~~