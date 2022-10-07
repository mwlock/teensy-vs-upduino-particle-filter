# Scripts

The original map provided with [mcl](https://github.com/debbynirwan/mcl) is seen in ```resources/epuck_world_map.pgm```. Initially, we created a script to convert the original PGM file to the neccessary C string to load into the Teensy. After some testing, it was found that this map was not entirely accurate. We instead created our own ```generate_map.py``` to create a more accurate map and produce the neccessary PGM map and C string.

## PGM to C Array

```
python pgm_to_array.py
```

This will export the data to ```map.h```.

## generate_map.py

```
python3 generate_map.py
```