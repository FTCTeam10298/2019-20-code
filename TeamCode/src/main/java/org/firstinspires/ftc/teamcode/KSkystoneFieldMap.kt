package org.firstinspires.ftc.teamcode

 class KSkystoneFieldMap {

    private var field = Int [144] [144]
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
        for (i: Int= 0; i < field.length; i++) {
            for (j: Int= 0; j < field[i].length; j++) {
                field[i][j] = 0
            }
        }

        //filling in foundations
        for (i: Int = 4; i < 39; i++) {
            for (int j = 47; j < 66; j++) {
            field[i][j] = 3;
        }

            for (int j = 78; j < 97; j++) {
            field[i][j] = 3;
        }
        }

        //filling in bridge posts
        for (int i = 63; i < 81; i++) {
            for (int j = 47; j < 48; j++){
            field[i][j] = 3;
        }

            for (int j = 96; j < 97; j++){
            field[i][j] = 3;
        }
        }

        //filling in quarry
        for (int i = 95; i < field.length; i++) {
            for (int j = 47; j < 52; j++){
            field[i][j] = 3;
        }

            for (int j = 92; j < 97; j++){
            field[i][j] = 3;
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