#deduction of the last block if they have all been identified..(is it rigorous?)
Rule {
  X, Y, T
  { (block X) (block Y) (id T) (identified X) (identified Y)! (NOT_OBSERVABLE is Y T)}
  { (identified Y) (is Y T) (NOT_OBSERVABLE is Y T)!}
}

