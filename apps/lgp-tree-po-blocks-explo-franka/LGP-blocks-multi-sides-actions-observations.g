# Check
DecisionRule check {
  X, Y, Z
  { (block X) (side Y) (identified X)! (bottom_facing Y)! (flipped Z) (hand_empty)} # remove bottom_facing
  { (in_sight Y) komoCheck(X, Y, Z)=1. }
}

# Pick-up
DecisionRule pick-up {
  X, Y, Z
  { (block X) (location Y) (on_table X Y) (flipped Z) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! komoPickUp(X Y Z)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y, Z, W
  { (block X) (location Y) (holding X) (flipped Z) (identified X) (colored W) }
  { (holding X)! (hand_empty) (on_table X Y Z) (orientation_corrected) komoPutDown(X Y Z W)=1. }
}

# Put-down-flipped
DecisionRule put-down-flipped {
  X, Y
  { (block X) (location Y) (holding X) (flipped FALSE) (NOT_OBSERVABLE colored side_5) }
  { (holding X)! (hand_empty) (bottom_facing side_5)! (bottom_facing side_3) (flipped TRUE) (orientation_corrected) (on_table X Y) komoPutDown(X Y)=1. }
}

### Rules / Observation Model
#Observation model
Rule {
  X, Y
  { (block X) (side Y) (NOT_OBSERVABLE colored Y) (in_sight Y) }
  { (in_sight Y)! (colored Y) (NOT_OBSERVABLE colored Y)! (identified X)}
}

