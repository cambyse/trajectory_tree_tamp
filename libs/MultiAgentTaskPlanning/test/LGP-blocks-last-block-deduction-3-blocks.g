#deduction of the last block if they have all been identified..(is it rigorous?)
Rule {
  X, Y, Z, T
  { (block X) (block Y) (block Z) (id T) (identified X) (identified Y) (identified Z)! (UNEQUAL X Y) (UNEQUAL X Z) (UNEQUAL Y Z) (NOT_OBSERVABLE is Z T)}
  { (identified Z) (is Z T) (NOT_OBSERVABLE is Z T)!}
}

