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
block_a  #block identifier
block_2
block_b  #block identifier
tableC


## initial state
START_STATE { (table tableC) 
(block block_1) (id block_a) (block block_2) (id block_b) 
(location tableC)
(UNEQUAL block_1 block_2)
(clear block_1) (clear block_2) (clear tableC)
(on_table block_1 tableC) (on_table block_2 tableC)
(hand_empty) 

#(identified block_1)
(identified block_2)
(is block_2 block_b)
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE is block_1 block_a)
}
}

BELIEF_START_STATE{ 
{
()=1.0
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
#Include = 'LGP-blocks-actions-no-observations.g'
Include = 'LGP-blocks-actions-observations.g'
