This is the readme file of the Python ILK-Generator tool.

The ILK-Generator takes a robot model and a user query and generates an
imperative model of the solver requested in the query, such as a forward
position kinematics algorithm.

The solver model can then be compiled (by the companion tool ILK-Compiler) into
actual code.

The ILK-Compiler requires Python **3**.

# Dependencies

- The [robot-model tools](https://github.com/mfrigerio17/robot-model-tools)

- [Mako](http://www.makotemplates.org)

- [NumPy](http://www.numpy.org)

- [YAML](http://pyyaml.org)


# Running the tool

Make sure the robot-model tools are in your `PYTHONPATH`, and install the other
dependencies (e.g. via `pip3 install`).

Then, run the tool from this folder with:

```
./ilkgen.py --help
```

Sample robot models working with the tool are available in the robot-model tools
repository (see [dependencies](#Dependencies) above). A sample query is available in
`sample/queries/`. For example:

```
./ilkgen.py --urdf <robot-model tools root>/sample/models/ur5/ur5.urdf
  --query sample/queries/ur5-simple.yaml --output-dir /tmp/ilkgen/ur5
```

