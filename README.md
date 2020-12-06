d0: position  
d1: velocity  

sim1: pybullet  
sim2: z3 (using equations of motion)  

## Run the following files in order

1. `pendulum_manipulator.py`: This file loads 2 single-link URDFs and is run `n` times. The position and velocity at each time-step is written to a file `position_velocity_pybullet_data.txt`. The format of the data written is as follows:
```
{
  "0": {
    "position": 1,
    "velocity": 0
  },
  "179": {
    "position": 1.0125164889847702,
    "velocity": 0.37573403437650427
  },
  "536": {
    "position": 1.0552359296886857,
    "velocity": 0.8064906280500382
  },
  "894": {
    "position": 1.1087021975111322,
    "velocity": 1.143005540634034
  }
}
```

The position and velocity at time-step 0, 179, 536 and 894 are given above.

2. `pendulum_z3_synthesize_mdp.py`: This file uses the data in the file generated above and using equations of motion, motion damping proxy is generated. This is done by using the following z3 constraint: `The difference between the velocity generated using equations of motions and velocity recorded for the pybullet model should be less than 0.5`. Once the mdp is generated, it is stored to a file: `mdp_list.txt`. The format of this file is as follows:
```
1.9998504291489958
1.9997380314304503
1.9996091741698674
1.999508508526166
1.9995006679764413
1.9993568921816436
1.999348755695386
1.9992416713598171
```

3. `pendulum_z3_collect_data.py`: This file is run to finally collate the data generated in `Step 2` and this data is written to a file `position_velocity_z3_data.txt`. The motor damping proxy generated in `step 2` is used to generate the position and velocity of the pendulum by plugging it into the equations of motion. The format of this file is as follows:
```
{
  "0": {
    "position": 0,
    "velocity": -0.9998504291489958,
    "mdp": 1.9998504291489958
  },
  "179": {
    "position": 179,
    "velocity": 11.656213574036299,
    "mdp": 1.9997380314304503
  },
  "536": {
    "position": 715,
    "velocity": -514.7729785773156,
    "mdp": 1.9996091741698674
  },
  "894": {
    "position": 1609,
    "velocity": 430.9686954800673,
    "mdp": 1.999508508526166
  }
}
```
Note that the timesteps at which this data is collected is same as the ones at which data was collect from the pybullet models.

4. `pendulum_compare.py`: Run this file to load the original pybullet single link URDF model using the same data that was stored in `position_velocity_z3_pybullet.txt` . The model generated using z3 is also loaded from the `position_velocity_z3_data.txt`.

----
## Commands to run all the files:
```
python3 pendulum_manipulator.py
python3 pendulum_z3_synthesize_mdp.py
python3 pendulum_compare.py
```
