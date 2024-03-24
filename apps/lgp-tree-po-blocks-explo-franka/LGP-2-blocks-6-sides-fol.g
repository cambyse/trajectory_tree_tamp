Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
block
side
location
table
id	    # block identifier

in_sight    # block identification part is visible
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear
identified  # object X as been identified, the agent knows which block it is
is
colored     # object colored side
bottom_facing
needs_flipping 
flipped
orientation_correct
observed    # observed X means that the side X has been observed
object_put_down

# keyword
NOT_OBSERVABLE
UNEQUAL
TRUE
FALSE

## constants
block_1
block_a  #block identifier
block_2
block_b  #block identifier
side_0
side_1
side_2
side_3
side_4
side_5
tableC


## initial state
START_STATE { 
(table tableC) 
(location tableC)
(side side_0) (side side_1) (side side_2) (side side_3) (side side_4) (side side_5)
(bottom_facing side_5)
(block block_1) (id block_a) (block block_2) (id block_b) 
(UNEQUAL block_1 block_2)
(clear block_1) (clear block_2) (clear tableC)
(on_table block_1 tableC) (on_table block_2 tableC)
(hand_empty) 
}

EVENTUAL_FACTS{ 
## 1 is a
# 0
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_0)
}
# 1
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_1)
}
# 2
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_2)
}
# 3
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_3)
}
# 4
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_4)
}
# 5
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_a)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE colored block_2 side_5)
}
## 1 is b
# 0
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_0)
}
# 1
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_1)
}
# 2
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_2)
}
# 3
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_3)
}
# 4
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_4)
}
# 5
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_0)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_1)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_2)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_3)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_4)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
{
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE colored block_1 side_5)
(NOT_OBSERVABLE is block_2 block_a)
(NOT_OBSERVABLE colored block_2 side_5)
}
}

BELIEF_START_STATE{ 
## 1 is a
# 0
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 1
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 2
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 3
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 4
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 5
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
## 1 is b
# 0
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 1
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 2
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 3
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 4
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
# 5
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
{
()=0.013888889
}
}

### Termination RULES 
Rule {
  { (on block_a block_b) (hand_empty) } # 
  { (QUIT) }
}

### Reward
REWARD {
}

### Tasks definitions
Include = 'LGP-multi-blocks-multi-sides-actions-observations.g'
#Include = 'LGP-blocks-last-block-deduction-2-blocks.g'
