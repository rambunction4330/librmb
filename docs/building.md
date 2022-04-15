# Building

```bash
# If you haven't installed the toolchain yet:
$ ./gradlew installRoboRioToolchain

# To build:
$ ./gradlew build

# To build docs
$ ./gradlew docs
# or
$ doxygen docs/Doxyfile
```


## Notes
* If you are on windows, replace gradlew with gradlew.bat


# Adding to a project

see https://docs.gradle.org/current/userguide/native_software.html#sec:project_dependencies

essentially, in your sources.cpp, in your main project, you need to add something like
```groovy
sources.cpp {
    lib project: "locationOflibrmb", library: "librmb", linkage: "static or shared. one of the two"
}
```

and in your settings.gradle

```groovy
include 'locationOflibrmb'
```

