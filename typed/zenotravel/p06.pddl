(DEFINE (PROBLEM ZTRAVEL-2-5)
 (:DOMAIN ZENO-TRAVEL)
 (:OBJECTS PLANE1 - AIRCRAFT
           PLANE2 - AIRCRAFT
           PERSON1 - PERSON
           PERSON2 - PERSON
           PERSON3 - PERSON
           PERSON4 - PERSON
           PERSON5 - PERSON
           CITY0 - CITY
           CITY1 - CITY
           CITY2 - CITY
           CITY3 - CITY
           FL0 - FLEVEL
           FL1 - FLEVEL
           FL2 - FLEVEL
           FL3 - FLEVEL
           FL4 - FLEVEL
           FL5 - FLEVEL
           FL6 - FLEVEL)

 (:INIT
  (AT PLANE1 CITY2)
  (FUEL-LEVEL PLANE1 FL5)
  (AT PLANE2 CITY1)
  (FUEL-LEVEL PLANE2 FL3)
  (AT PERSON1 CITY0)
  (AT PERSON2 CITY0)
  (AT PERSON3 CITY3)
  (AT PERSON4 CITY1)
  (AT PERSON5 CITY2)
  (NEXT FL0 FL1)
  (NEXT FL1 FL2)
  (NEXT FL2 FL3)
  (NEXT FL3 FL4)
  (NEXT FL4 FL5)
  (NEXT FL5 FL6))
 (:GOAL
  (AND (AT PERSON1 CITY3)
       (AT PERSON2 CITY1)
       (AT PERSON3 CITY3)
       (AT PERSON4 CITY3)
       (AT PERSON5 CITY1))))