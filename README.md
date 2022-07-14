# MeshAnimation
### 3D Mesh Animator using Mesh Skinning 
Made by Evan Azari and Chuckwuemeka Ubakanma

## INSTRUCTIONS TO RUN:
First, unzip ```ext.zip```
Then, in the main folder, run the following commands in the terminal:

```
mkdir build
cd build
cmake ..
make -j 8
```

This creates the executable Rast. You can render a scene by typing:

```
./Rast mosh_cmu_8806_f_lbs_10_207_0_v1.0.2.fbx
```
or 
```
./Rast mosh_cmu_7516_f_lbs_10_207_0_v1.0.2.fbx
```
or 
```
./Rast mosh_cmu_0511_f_lbs_10_207_0_v1.0.2.fbx
```
Once running, press the right/left arrow keys to speed/slow down the animation. Press 'R' to reset the animation.
