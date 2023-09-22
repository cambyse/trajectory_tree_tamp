Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
block
location
table
id	    # block identifier

in_sight    # block identification part is visible
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear
identified  # object X as been identified, the agnt knows which block it is
is

# keyword
NOT_OBSERVABLE
UNEQUAL

## constants
block_1
block_r  #block identifier
block_2
block_g  #block identifier
block_3
block_b  #block identifier
tableC


## initial state
START_STATE { (table tableC) 
(block block_1) (id block_r) (block block_2) (id block_g) (block block_3) (id block_b) 
(location tableC)
(UNEQUAL block_1 block_2) (UNEQUAL block_1 block_3) (UNEQUAL block_2 block_3)
(clear block_3) (clear block_2) (clear tableC)
(on_table block_1 tableC) (on_table block_2 tableC) (on block_3 block_1)
(hand_empty) 
}


EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE is block_1 block_r)
(NOT_OBSERVABLE is block_2 block_g)
(NOT_OBSERVABLE is block_3 block_b)
}
{
(NOT_OBSERVABLE is block_1 block_r)
(NOT_OBSERVABLE is block_3 block_g)
(NOT_OBSERVABLE is block_2 block_b)
}

{
(NOT_OBSERVABLE is block_2 block_r)
(NOT_OBSERVABLE is block_1 block_g)
(NOT_OBSERVABLE is block_3 block_b)
}
{
(NOT_OBSERVABLE is block_2 block_r)
(NOT_OBSERVABLE is block_3 block_g)
(NOT_OBSERVABLE is block_1 block_b)
}

{
(NOT_OBSERVABLE is block_3 block_r)
(NOT_OBSERVABLE is block_1 block_g)
(NOT_OBSERVABLE is block_2 block_b)
}
{
(NOT_OBSERVABLE is block_3 block_r)
(NOT_OBSERVABLE is block_2 block_g)
(NOT_OBSERVABLE is block_1 block_b)
}
}

BELIEF_START_STATE{ 
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
}

### Termination RULES 
Rule {
  { (on block_g block_r) (on block_b block_g) (hand_empty) } # 
  { (QUIT) }
}

### Reward
REWARD {
}

### Tasks definitions
Include = 'LGP-blocks-actions-observations.g'
Include = 'LGP-blocks-last-block-deduction-3-blocks.g'
