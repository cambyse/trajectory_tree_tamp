#deduction of the last block if they have all been identified..(is it rigorous?)
Rule {
  B, X, Y, Z, T, U, V
  { (block B) (observed X) (observed Y) (observed Z) (observed T) (observed U) (NOT_OBSERVABLE colored V) (identified B)! (UNEQUAL X Y) (UNEQUAL X Z) (UNEQUAL X T) (UNEQUAL X U) (UNEQUAL X V) (UNEQUAL Y X) (UNEQUAL Y Z) (UNEQUAL Y T) (UNEQUAL Y U) (UNEQUAL Y V) }
  { (identified Z) (is Z T) (NOT_OBSERVABLE is Z T)!}
}

