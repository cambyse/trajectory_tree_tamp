### Actions
# Check
DecisionRule check {
  X, Y
  { (block X) (side Y) (identified X)!  (observed X Y)! (hand_empty)} # (bottom_facing Y)!
  { (in_sight X Y) (observed X Y) komoCheck(X, Y)=1. }
}

# Pick-up
DecisionRule pick-up {
  X, Y
  { (block X) (location Y) (clear X) (on_table X Y) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoPickUp(X Y)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y
  { (block X) (location Y) (holding X) }
  { (holding X)! (hand_empty) (clear X) (on_table X Y) komoPutDown(X Y)=1. }
}

# Stack
DecisionRule stack {
  X, Y
  { (block X) (block Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear X) (clear Y)! (on X Y) komoStack(X Y)=1. }
}

# Unstack
DecisionRule unstack {
  X, Y
  { (block X) (block Y) (clear X) (on X Y) (hand_empty) }
  { (on X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoUnStack(X Y)=1. }
}

### Observation Model
# Observation model (side identification)
Rule {
  X, Y, Z
  { (block X) (side Y) (id Z) (NOT_OBSERVABLE colored X Y) (in_sight X Y) (NOT_OBSERVABLE is X Z)}
  { (in_sight X Y)! (colored X Y) (NOT_OBSERVABLE colored X Y)! (is X Z) (NOT_OBSERVABLE is X Z)! (identified X)}
}

# Observation model (block identification)
#Rule {
#  X, Y
#  { (block X) (id Y) (NOT_OBSERVABLE is X Y) (in_sight X) }
#  { (in_sight X)! (is X Y) (identified X)  (NOT_OBSERVABLE is X Y)!}
#}

# Other Rules
#Apply identification to the ON
Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y) }
  { (on Z T)}
}

Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y)! (on Z T)}
  { (on Z T)!}
}

Rule { # remove in sight
  X, Y
  { (block X) (side Y) (colored Y)! (in_sight X Y) }
  { (in_sight X Y)!}
}

Rule { # remove in sight
  X, Y
  { (block X) (side Y) (NOT_OBSERVABLE colored Y)! (in_sight X Y) }
  { (in_sight X Y)!}
}

# Helpers
Rule {
  { (colored side_1) }
  { (orientation_correct)}
}

Rule {
  { (colored side_4) (flipped FALSE) }
  { (needs_flipping)}
}

Rule {
  { (colored side_5) (flipped FALSE) }
  { (needs_flipping)}
}

# last side deduction
#Rule {
#  { (observed side_0) (observed side_1) (observed side_2) (observed side_3) (observed side_4) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_5)! (colored side_5)}
#}

#Rule {
#  { (observed side_0) (observed side_1) (observed side_2) (observed side_3) (observed side_5) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_4)! (colored side_4)}
#}

#Rule {
#  { (observed side_0) (observed side_1) (observed side_2) (observed side_4) (observed side_5) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_3)! (colored side_3)}
#}

#Rule {
#  { (observed side_0) (observed side_1) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_2)! (colored side_2)}
#}

#Rule {
#  { (observed side_0) (observed side_2) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_1)! (colored side_1)}
#}

#Rule {
#  { (observed side_1) (observed side_2) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
#  { (NOT_OBSERVABLE colored side_0)! (colored side_0)}
#}
