# Runnymede Robotics 2025 Swerve

This repository is the code that operates FRC Team 1310's 2025 swerve drive.

## Local Development

### Code Formatting

This project uses [Google Java Format](https://github.com/google/google-java-format) to format code. Please install the
appropriate plugin for your IDE:

* IntelliJ IDEA: [google-java-format](https://plugins.jetbrains.com/plugin/8527-google-java-format)
    * And Add the following to your VM options (In `Help | Edit Custom VM Options`):

```
--add-exports=jdk.compiler/com.sun.tools.javac.api=ALL-UNNAMED
--add-exports=jdk.compiler/com.sun.tools.javac.code=ALL-UNNAMED
--add-exports=jdk.compiler/com.sun.tools.javac.file=ALL-UNNAMED
--add-exports=jdk.compiler/com.sun.tools.javac.parser=ALL-UNNAMED
--add-exports=jdk.compiler/com.sun.tools.javac.tree=ALL-UNNAMED
--add-exports=jdk.compiler/com.sun.tools.javac.util=ALL-UNNAMED
```

* VSCode: [google-java-format](https://marketplace.visualstudio.com/items?itemName=JoseVSeb.google-java-format-for-vs-code)
    * This will be suggested in `Extensions` when you open the project in VSCode.


* Eclipse: [google-java-format](https://marketplace.eclipse.org/content/google-java-format)

## Dependencies

In addition to the standard WPILib dependencies, this project relies on Runnymede
Robotics' [Runnymede Swerve Library](https://github.com/RunnymedeRobotics1310/RunnymedeSwerve).

The library can be built locally and installed, or it can be accessed from Maven Central.

## Local Development

If you are developing the swerve robot locally and don't need anything other than the current version of
RunnymedeSwerve, you don't need to do anything. You can use the latest version of the library from Maven Central, and
set it in your build.gradle file like this:

```gradle
    implementation 'ca.team1310:swerve:2.0.6'
```

If you are updating the RunnymedeSwerve library at the same time, then follow the instructions for local building in
RunnymedeSwerve and update the version number in the build.gradle file to match your locally installed version. This
codebase will read from your lmaven local repository to find the installed working copy of the library.

The first time you build against a new library, run the following:

```bash
./gradlew clean && \
./gradlew compileJava
```

which will fetch the latest version from your local repository.