# RobotecWatchdogTools

This tool gem allows setting up runtime checks to prevent the project from starting if some runtime requirements are not met.

The default O3DE behavior is to skip non-loading dynamic modules and simply not serve their functionality. Most often it is desired behavior.
However, if the functionality provided by non-loading module is a **hard requirement** we want to shorten the feedback loop and inform the reader that there are issue with the configuration of their environment immediately.

The gem has minimal requirements, and it's on purpose: we want to minimize the situations when gem fails to load and can't serve its purpose.

## How to use:

- Add a gem to your project.
- Create a setreg file with Watchdog rules in your project's registry.
- Good rule of thumb is to maintain two files: one for the Editor and a second for GameLauncher.

## Example:

Below is an example how to prevent Editor from starting if `ROS2.Editor` or `my_example_gem.Editor` is not loaded. 

Create a setreg file in your project's Registry:

```bash
cd ${MY_PROJECT_ROOT}/Registry
touch watchdog.editor.setreg
``` 

with the following content:

```json
{
    "O3DE": {
        "Watchdog": {
            "RequiredModules": [
                "ROS2.Editor",
                "my_example_gem.Editor"
            ]
        }
    }
}
```

## Current functionality:

Currently, only implemented rule is enumerating **dynamic** modules loaded by engine and comparing it with provided list of required ones.
This by itself helps to quickly catch multiple issues like non-sourced environment, missing required packages or messed up symlinks for 3rd party dynlibs.

I eagerly anticipate contributions that include additional rules.

### Rules:
`O3DE/Watchdog/RequiredModules` - it accepts a list of required dynamic modules. Note that both prefix (lib) and suffix (.so) are optional

