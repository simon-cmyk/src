(define (problem prob5)
 (:domain domain-tms-2-3-light)
 (:objects 
 kiln0 - kiln8
 kiln1 - kiln20
 pone0 pone1 pone2 pone3 pone4 pone5 pone6 pone7 pone8 pone9 pone10 pone11 pone12 pone13 pone14 pone15 pone16 pone17 pone18 pone19 pone20 pone21 pone22 pone23 pone24 pone25 pone26 pone27 - piecetype1
 ptwo0 ptwo1 ptwo2 ptwo3 ptwo4 ptwo5 ptwo6 ptwo7 ptwo8 ptwo9 ptwo10 ptwo11 ptwo12 ptwo13 ptwo14 ptwo15 ptwo16 ptwo17 ptwo18 ptwo19 ptwo20 ptwo21 ptwo22 ptwo23 ptwo24 ptwo25 ptwo26 ptwo27 ptwo28 ptwo29 ptwo30 ptwo31 ptwo32 ptwo33 ptwo34 ptwo35 ptwo36 ptwo37 ptwo38 ptwo39 ptwo40 ptwo41 - piecetype2
 pthree0 pthree1 pthree2 pthree3 pthree4 pthree5 pthree6 pthree7 pthree8 pthree9 pthree10 pthree11 pthree12 pthree13 pthree14 pthree15 pthree16 pthree17 pthree18 pthree19 pthree20 pthree21 pthree22 pthree23 pthree24 pthree25 pthree26 pthree27 pthree28 pthree29 pthree30 pthree31 pthree32 pthree33 pthree34 pthree35 pthree36 pthree37 pthree38 pthree39 pthree40 pthree41 pthree42 pthree43 pthree44 pthree45 pthree46 pthree47 pthree48 pthree49 pthree50 pthree51 pthree52 pthree53 pthree54 pthree55 pthree56 pthree57 pthree58 pthree59 pthree60 pthree61 pthree62 pthree63 pthree64 pthree65 pthree66 pthree67 pthree68 pthree69 - piecetype3
)
 (:init 
  (energy)
)
 (:goal
  (and
     (baked-structure ptwo14 pone23)
     (baked-structure pone11 ptwo3)
     (baked-structure pthree45 pthree60)
     (baked-structure ptwo15 pthree55)
     (baked-structure pthree5 ptwo27)
     (baked-structure pthree44 pthree33)
     (baked-structure pthree30 pthree66)
     (baked-structure pthree58 ptwo29)
     (baked-structure ptwo37 pone4)
     (baked-structure pthree11 ptwo16)
     (baked-structure pthree13 pone24)
     (baked-structure pthree24 pthree48)
     (baked-structure ptwo17 pone21)
     (baked-structure ptwo4 pthree41)
     (baked-structure ptwo36 pone12)
     (baked-structure pthree42 pthree67)
     (baked-structure pthree34 pthree18)
     (baked-structure pthree61 pthree21)
     (baked-structure ptwo23 pone17)
     (baked-structure pthree26 ptwo39)
     (baked-structure pone2 ptwo33)
     (baked-structure ptwo26 ptwo11)
     (baked-structure pone8 pone5)
     (baked-structure ptwo30 ptwo35)
     (baked-structure pthree7 pthree25)
     (baked-structure pthree62 pone27)
     (baked-structure pone1 pone15)
     (baked-structure pthree69 pthree36)
     (baked-structure ptwo10 ptwo22)
     (baked-structure pone13 pone16)
     (baked-structure pthree12 ptwo19)
     (baked-structure pthree39 pone18)
     (baked-structure pthree43 ptwo1)
     (baked-structure ptwo41 ptwo38)
     (baked-structure pthree31 pthree47)
     (baked-structure pthree6 ptwo2)
     (baked-structure ptwo13 pthree23)
     (baked-structure pthree56 pthree0)
     (baked-structure pthree14 pthree52)
     (baked-structure pthree17 pthree68)
     (baked-structure pthree51 pthree19)
     (baked-structure ptwo6 pone25)
     (baked-structure pthree63 pthree32)
     (baked-structure pone3 ptwo31)
     (baked-structure pthree9 pone19)
     (baked-structure ptwo7 ptwo0)
     (baked-structure pthree28 ptwo24)
     (baked-structure ptwo34 pthree50)
     (baked-structure ptwo9 ptwo12)
     (baked-structure pthree2 pone7)
     (baked-structure ptwo20 ptwo8)
     (baked-structure pthree37 pthree3)
     (baked-structure pthree29 ptwo32)
     (baked-structure pone22 pthree38)
     (baked-structure pthree8 pthree35)
     (baked-structure pthree46 pone14)
     (baked-structure pthree54 pthree22)
     (baked-structure pthree53 ptwo18)
     (baked-structure ptwo21 pone9)
     (baked-structure ptwo5 pone10)
     (baked-structure pthree40 ptwo28)
     (baked-structure pthree16 pone6)
     (baked-structure pone20 pthree15)
     (baked-structure pthree4 pthree27)
     (baked-structure pone0 pthree59)
     (baked-structure pthree1 pone26)
     (baked-structure pthree65 pthree64)
     (baked-structure ptwo25 pthree10)
     (baked-structure pthree57 pthree49)
     (baked-structure ptwo40 pthree20)
))
 (:metric minimize (total-time))
)
