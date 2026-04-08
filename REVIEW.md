#### _first of all, some of these packages don't build off a clean install:_

```
CMake Error at CMakeLists.txt:10 (find_package):
  By not providing "Findnav2_bringup.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "nav2_bringup", but CMake did not find one.

  Could not find a package configuration file provided by "nav2_bringup" with
  any of the following names:

    nav2_bringupConfig.cmake
    nav2_bringup-config.cmake

  Add the installation prefix of "nav2_bringup" to CMAKE_PREFIX_PATH or set
  "nav2_bringup_DIR" to a directory containing one of the above files.  If
  "nav2_bringup" provides a separate development package or SDK, be sure it
  has been installed.
```

### File organization

---

- There's a lot of debug files laying around (`.pdf`, `.txt`, `.gv`, etc).
- These files should either be untracked or at least put into their own directory (`/debug` for example)
  - You could also add this debug directory to `.gitignore` for easier management
- `.vscode` should definitely be untracked and added to `.gitignore`. These configurations are usually machine or user specific

### Packages

---

- Package names should be consistent, your packages should follow the pattern of `nyurdt_*`. The following packages do not adhere:
  - april
    - _Is this supposed to be `apriltag`?_
  - pathplanning
  - rdt_nav2_bringup
  - rtab
  - rtabmap_ros
- It's unclear to me which of these are forks and which are your own custom packages
- Packages should have a README describing what their purpose, usage, nodes, and any interfaces
  - Some packages are not entirely clear at a glance
  - `nyurdt_interfaces` should definitely have a README descibing the interfaces it provides
- Your `package.xml` files should _**definitely be updated**_
  - Those files are basically a detail what is required to build your package
  - It should contain a list of dependencies required for building and executing
    - For example, if you depend on numpy to run some code, `<depend>python3-numpy</depend>`
  - If these files are kept up to date, then anyone can make a clean install and know what dependencies are required to get your package working as intended

### Commits

---

- This is 100% on me for saying this WAY too late (my bad) but generally, you want to try to keep your commit history clean. This means:
  - Clear names. For example, if I would committing something like...
    - `[AprilTag] Implemented calibration script`
    - `[GZ] Fixed LiDAR simulation data`
  - Smaller commits
    - Better debugging (and easier to rollback if need be)
    - Quicker and easier to review
    - Helps keep a readable project history
    - Reduces the chance of merge conflicts
  - Squashing commits before merging
    - If you have a pull request that has 6 or 7 different commits, you should squash them into one before merging them
    - Usually, pull requests SHOULD only cover one feature/bug at a time, so squashing separate commits into one doesn't muddy the purpose of the commit
    - Also GitHub just has a "Squash and Merge" button built in for pull requests, so you can avoid manually rebasing most of the time
- There are many different practices for this, so it ultimately is up to you (or your boss) on how things are maintained. But in general, a clean commit history will _always_ help
- Real life example: [LLVM](https://github.com/llvm/llvm-project/commits/main/)
  - Every commit is descriptive, and comes from a squashed pull request
  - bonus: [my pull requests for llvm-zorg ](https://github.com/llvm/llvm-zorg/pulls?q=is%3Apr+author%3Ajriv01)

### Forking

---

- If you are using other repositories (such as [rtabmap_ros](https://github.com/introlab/rtabmap_ros)), it is generally anti-pattern to copy + paste their source code and then include those directories in the git ignore
- Admittedly I don't know the 100% correct way to handle this, but I would fork their repository and then take advantage of git submodules to use them
  - This is good if you need to make modifications to the original work, and it keeps the commit history clean (very obvious where open-source ends and where your contributions begin)
  - git submodules allow you to reference other repositories within your own repository
    - https://github.blog/open-source/git/working-with-submodules/
    - https://git-scm.com/book/en/v2/Git-Tools-Submodules
  - There's also [vcstool](https://github.com/dirk-thomas/vcstool), but I've never used this one
    - ros2 _itself_ uses this, not the [ros2.repos](https://github.com/ros2/ros2/blob/rolling/ros2.repos) file
- I also can't tell what's coming from open-source or what's original work? (due to naming & messy commit history)
  - rdt_nav2_bringup
  - rtab
  - rtabmap_ros
  - pathplanning
  - april

### Licensing

---

- Random apache license in `nyurdt_autonomy`???
- Depending on who you fork from, they may have some licensing requirements
  - rtabmap_ros is licensed under BSD 3-Clause, which has some requirements for usage (include the license and copyright with your modified work)
  - This is honestly a very industry thing so, more than anything, it's just something to keep in mind
