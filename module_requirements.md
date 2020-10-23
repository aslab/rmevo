# Module requirements

Currently, the imported *.sdf* files containing modules have to meet the following requirements:

- There can be only one <link> or <joint> tag in each file.
- There can be only one <visual>, one <collision> and one <inertial> in each link.
- <joint>s cant have any of those properties.