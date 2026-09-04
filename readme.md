This is the ILK-Generator command line tool.

The ILK-Generator takes a robot model and a user query and generates an
imperative model of the solver requested in the query, such as a forward
position kinematics algorithm.

The solver model can then be compiled into actual code, using
the companion tool [ILK-Compiler](https://github.com/ESROCOS/ilk-compiler)

More background on the rationale and the methodology behind this approach
is available in this [conference paper](https://ieeexplore.ieee.org/document/8675586):

Marco Frigerio, Enea Scioni, Pawel P. Pazderski and Herman Bruyninckx,
*"Code Generation from Declarative Models of Robotics Solvers"*, in
Third IEEE International Conference on Robotic Computing (IRC), 2019,
pp. 369-372.


# Running the tool

Clone the repository and install using `pip`:
```
pip install .
```

Then, run the tool with:

```
ilkgen --help
```

## Dependencies
See `pyproject.toml`.

- [robot-model tools](https://github.com/mfrigerio17/robot-model-tools)

- [Mako](http://www.makotemplates.org)

- [NumPy](http://www.numpy.org)

- [YAML](http://pyyaml.org)


# Sample

Sample robot models working with the tool are available in the
[robot-model tools](https://github.com/mfrigerio17/robot-model-tools)
repository (see [dependencies](#Dependencies) above). A sample query is available in
`sample/queries/`.

After installation, try for example:

```
ilkgen <robot-model tools root>/sample/models/ur5/ur5.urdf
  --query sample/queries/ur5-simple.yaml --output-dir /tmp/ilkgen/ur5
```

# License
© 2019 KU Leuven (Marco Frigerio, Enea Scioni)

This package is distributed under the 'BSD-2-Clause' license.
See the `LICENSE` file for more details.


