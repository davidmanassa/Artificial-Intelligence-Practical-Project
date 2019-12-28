
    Portas
    A(r1, r2, x1, y1) ---> B(r2, r3, x2, y2) ---> ...


encontrar caminho A -> Z
percorrer portas 
    se A -> B
        guardar A->B
    
    se B -> K
        guardar A->B->K

    ...

    se K == Z       (criterio de paragem)
        retorna lista A->...->Z

(   !Apenas retorna um caminho!  )



toCheck
checked
while true
    percorrer toCheck
        se A (toCheck) -> B
            se checked B
                ()
            se ñ em checked B & ñ em toCheck B
                add toCheck B




