Include = 'data/franka/franka.ors'

shape franka_hand (panda_body_8){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0.0 0 -0.12) d(0 0 0 1)> }

body table{ type=9, X=<T t(0.55 0 0.33)>, size=[0.8 1.5 .04 .01], color=[.2 .2 .7] }

body block_1 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	

joint (table block_1) { from=<T t( .0 .3 .07 ) t(0 0 0)> to=<T > type=10 }
