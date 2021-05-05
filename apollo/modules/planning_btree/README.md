# Behaviour Tree Planning module

This module implements behaviour tree planning algorighms

## Folder descriptions

### Behaviours

Contains Behaviour Tree node definitions

### Common

Some utility classes that are used across all other classes in the planning module. The most important classes are:

- BTreeFrame
- DynamicReferenceLine

### Conf

Folder with configuration files for the component

### Contexts

Contains context definitions

### Dag

Folder with a dag file to start the component

### Launch

Folder with a launch file to launch the component

### Math

This folder contains tools that are used as utils for the reference line smoothing and piecewise-jerk path optimization

### Proto

Protobuf definitions for the component

### Reference Line

This folder contains the definition of the reference line class, reference line smoothers, and the reference line provider wrapper around the reference line

The main purpose of the reference line provider is to generate and smooth reference lines

### Stages

Contains stage definitions

### Traffic rules

Contains traffic rule definitions