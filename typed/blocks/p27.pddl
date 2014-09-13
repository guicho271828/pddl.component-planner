(DEFINE (PROBLEM BLOCKS-13-1)
 (:DOMAIN BLOCKS)
 (:OBJECTS I - OBJECT
           M - OBJECT
           G - OBJECT
           H - OBJECT
           L - OBJECT
           A - OBJECT
           C - OBJECT
           D - OBJECT
           E - OBJECT
           K - OBJECT
           F - OBJECT
           B - OBJECT
           J - OBJECT)

 (:INIT
  (CLEAR J)
  (CLEAR B)
  (ONTABLE F)
  (ONTABLE K)
  (ON J E)
  (ON E D)
  (ON D C)
  (ON C A)
  (ON A L)
  (ON L H)
  (ON H G)
  (ON G M)
  (ON M I)
  (ON I F)
  (ON B K)
  (HANDEMPTY))
 (:GOAL
  (AND (ON D A)
       (ON A E)
       (ON E L)
       (ON L M)
       (ON M C)
       (ON C J)
       (ON J F)
       (ON F K)
       (ON K G)
       (ON G H)
       (ON H I)
       (ON I B))))