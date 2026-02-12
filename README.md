# Solidean - Hello Godot

Prebuilt binaries for this example: https://solidean.com/download/godot-demo/

## Getting started (compiling yourself)

* download the Solidean Community Edition
  https://solidean.com/download/solidean/

* install `scons` 
  via python/pip

* run `python -m SCons target=template_release` 
  in `addons/godot-solidean/extern/godot-cpp` to create the c++ bindings 
  (everything generated is in the .gitignore)

* run `python -m SCons target=template_release solidean_path=C:/Users/John/solidean` 
  in the `addons/godot-solidean` folder to build the extension (DLLs)

* make sure to pass *your* solidean path here (where lang and lib folders reside). 
  Re-run this command whenever you modify anything in godot-solidean/src

* **make sure to put your own solidean.dll in the godot-solidean/bin folder**
  (NOTE: this folder is created when running scons in the `addons/godot-solidean` folder)

### Notes: 

* godot-cpp is on branch godot-4.5-stable (should match your Godot version!)
* only tested/built on Windows so far


## License

This example project (source code in this repository) is licensed under the MIT License.

It depends on the Solidean SDK, which is not covered by the MIT License.
Solidean binaries (e.g. solidean.dll) are distributed under their respective Solidean license, such as the Solidean Community Edition License.
