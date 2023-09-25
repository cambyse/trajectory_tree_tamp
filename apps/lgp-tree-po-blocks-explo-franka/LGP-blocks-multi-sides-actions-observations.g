### Actions
# Check
DecisionRule check {
  X, Y, Z
  { (block X) (side Y) (identified X)! (bottom_facing Y)! (observed Y)! (flipped Z) (hand_empty)} # remove bottom_facing ?
  { (in_sight Y) (observed Y) komoCheck(X, Y, Z)=1. }
}

# Pick-up
DecisionRule pick-up { # can be dead-end if picked up without identification!
  X, Y, Z, T
  { (block X) (location Y) (side T) (on_table X Y) (flipped Z) (colored T) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! komoPickUp(X Y Z T)=1. }
}

# Put-down
DecisionRule put-down { 
  X, Y, Z
  { (block X) (location Y) (side Z) (holding X) (identified X) (needs_flipping)! (colored Z) }
  { (holding X)! (hand_empty) (on_table X Y) (orientation_correct) (object_put_down) komoPutDown(X Y Z)=1. }
}

# Put-down-flipped
DecisionRule put-down-flipped {
  X, Y, Z
  { (block X) (location Y) (side Z) (holding X) (flipped FALSE) (needs_flipping) (colored Z)}
  { (holding X)! (hand_empty) (bottom_facing side_5)! (bottom_facing side_2) (flipped TRUE) (flipped FALSE)! (needs_flipping)! (orientation_correct) (object_put_down) (on_table X Y) komoPutDown(X Y Z)=1. }
}

### Rules / Observation Model
# Observation model
Rule {
  X, Y
  { (block X) (side Y) (NOT_OBSERVABLE colored Y) (in_sight Y) }
  { (in_sight Y)! (colored Y) (NOT_OBSERVABLE colored Y)! (identified X)}
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
Rule {
  { (observed side_0) (observed side_1) (observed side_2) (observed side_3) (observed side_4) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_5)! (colored side_5)}
}

Rule {
  { (observed side_0) (observed side_1) (observed side_2) (observed side_3) (observed side_5) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_4)! (colored side_4)}
}

Rule {
  { (observed side_0) (observed side_1) (observed side_2) (observed side_4) (observed side_5) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_3)! (colored side_3)}
}

Rule {
  { (observed side_0) (observed side_1) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_2)! (colored side_2)}
}

Rule {
  { (observed side_0) (observed side_2) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_1)! (colored side_1)}
}

Rule {
  { (observed side_1) (observed side_2) (observed side_3) (observed side_4) (observed side_5) (identified block_1)! }
  { (NOT_OBSERVABLE colored side_0)! (colored side_0)}
}
