# FDM Links

This is a blender extension that simplifies making flexible or articulated toys optimised for Fused Deposition Modeling 3d printing.

## Advantages

These links are specially designed for FDM printing.

* No reliance on layer lines for stability.
* No angles greater than 45 degrees.

These are the two biggest problems with flexible/articulated models commonly available. Because these new links do not rely on layer line strength,
other filaments that are usually too fragile for these types of models, like Silk PLA, are now usable. These links are also much less likely to
break than the interlocking ring link style that is so common, making the process of making and giving out 3d-printed toys much more enjoyable.

## Procedure

Here's how to use it:

1. Create a model
2. Add a link
3. Add a Boolean difference modifier on the model with the link as the Object

## Requirements

This extension relies on the Manifold option for boolean operators, which was added in Blender 4.5.

This extension also makes some assumptions about your scene.

* Units: Metric
* Unit Scale: 0.001
* Unit Length: Millimeters

Although not required, it is recommended to also change the grid scale to 0.1 and the view clipping end to 10000 mm.

## Limitations

* Requires scene to be set up properly
* Greater vertical bending radius than horizontal, particularly at smaller sizes
* Only 0.4 mm nozzle diameters are supported (To use other sizes of nozzles, the model and links should be scaled accordingly.)
* Assumes 0.2 mm layer heights (Smaller layer heights should be fine, but probably not larger.)
* Assumes z=0 to be the bottom of the model and will not work correctly for models that extend below the z=0 plane
* Angles aren't exact

