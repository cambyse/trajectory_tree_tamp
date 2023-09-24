Include = 'data/franka/franka.ors'

shape franka_hand (panda_body_8){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0 0 -0.12) d(0 0 0 1)> }


body tableC{ type=9, X=<T t(0.5 0 0.2)>, size=[0.6 0.6 .04 .01], color=[.2 .2 .7] }

body block_1 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	
shape xx (block_1){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(-0.1 0 0.0)> }

joint (tableC block_1) { from=<T t(0.0 0 0.07) > type=10 }
 
BELIEF_START_STATE { 
{
 shape id_1(block_1) { type=9 rel=<T t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-90 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-180 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-270 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(90 0 1 0) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-90 0 1 0) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
}
}

