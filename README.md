# 31385 Autonomous robot systems Jan 2020

This git contains our solution for the 3-week project of Autonomous robot systems
Our project has the following structure
.
├── bin \
├── calib \
│      ├── data.debug \
│      ├── log \
│      ├── robot.conf \
│      ├── smr9_demo_ls_calib.dat \
│      ├── smr9_demo_odo_calib.dat \
│      └── smrxqptest \
├── log \
│       ├── 2020-01-14_00001.dat \
│       └── README.md \
├── Makefile \
└── src \
          ├── main.c \
          ├── missions.c \
          ├── missions.h \
          ├── motioncontroller.c \
          ├── motioncontroller.h \
          └── serverif.c \



#### Using git

Install git with

```sh
sudo apt install git
```

Download the git project with

```bash
git clone https://github.com/mthieden/autonomous_robots.git
```

To get the latest changes on the current branch use

```bash
git fetch --all
git pull
```

When creating a new branch use
```bash
git checkout -b branch_name
```
This create a new branch and changes the current branch the the new one.

To change the from one branch to another use
```bash
git checkout  branch_name
```


##### Adding changes made to git

To see current changes made, use
```bash
git status
```

Add changes with
```bash
git add filepath/filename.c
```
To add all changes made use
```bash
git add .
```
Commit the changes made with
```bash
git commit -m "helpful message about the changes made"
```
Push all changes commited with
```bash
git push
```

To add your changes to the master branch, firstly commit and push your changes on your branch. Then change branch to the master branch, get all the lastest changes with

```bash
git fetch --all
git pull
```
Then use
```bash
git merge branch_name
```
This will add all your changes to the masterbranch and latest release of the project

## Usage
Compiling the program require the smr-cl libraries, with these installed use
```bash
make
```
To run the program use
```bash
make run
```

To run the program with calibrations use
```bash
make run X
```
Where x is the number of the robot




