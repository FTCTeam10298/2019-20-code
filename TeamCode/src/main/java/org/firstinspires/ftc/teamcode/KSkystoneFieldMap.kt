package org.firstinspires.ftc.teamcode

 class KSkystoneFieldMap {

    private var field: Array<Array<Int>> = Array(144) {_ -> Array(144) {_ ->  0} }
    private var x: Int= 0
    private var y: Int= 0

    /** Constructor
     * @param xnot starting x position
     * @param ynot starting y position
     * */

    fun skystoneFieldMap (xnot: Int, ynot: Int) {
        //ensuring coordinates are within field bounds
        x = when {
            xnot > 72 -> 72
            xnot < -72 -> -72
            else -> xnot
        }

        y = when {
            ynot > 72 -> 72
            ynot < -72 -> -72
            else -> ynot
        }

        //creating empty field
        for (i in (0..field.size)) {
            for (j in (0..field[i].size)) {
                field[i][j] = 0
            }
        }

        //filling in foundations
        for (i in (0..39)) {
            for (j in (47..66)) {
                field[i][j] = 3
            }
            for (j in (78..97)) {
                field[i][j] = 3
            }
        }

        //filling in bridge posts
        for (i in (63..81)) {
            for (j in (47..48)){
                field[i][j] = 3
            }

            for (j in (96..97)){
                field[i][j] = 3
            }
        }

        //filling in quarry
        for (i in (95..field.size)) {
            for (j in (47..52)){
                field[i][j] = 3
            }

            for (j in (92..97)){
                field[i][j] = 3
            }
        }

        //checking coordinate legality
        if (field[-y+72][x+72] != 0){
            throw IllegalArgumentException("Coordinates must be in empty field")
        }
        else {
            field[72-y][x+72] = 1
        }
    }

    override fun toString(): String {
        return "$x, $y"
    }

    fun getX(): Int {
        return x
    }

     fun getY(): Int {
        return y
     }

    fun setX(newX: Int) {
        x = when {
            newX > 72 -> 72
            newX < -72 -> -72
            else -> newX
        }
    }

    fun setY(newY: Int) {
         var tempY: Int= newY
         if (newY > 72)
            tempY = 72
         else if (newY < -72)
            tempY = -72
         else if (field[-tempY+72][x+72] != 0)
            throw IllegalArgumentException("Coordinates must be in empty field")
         else {
            field[-y+72][x+72] = 0
            y = tempY
            field[-y+72][x+72] = 1
        }
    }

}