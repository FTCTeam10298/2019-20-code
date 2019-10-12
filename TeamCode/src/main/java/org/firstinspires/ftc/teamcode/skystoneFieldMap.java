package org.firstinspires.ftc.teamcode;

public class skystoneFieldMap {

    private int [] [] field = new int [144] [144];
    private int x;
    private int y;

    /** Constructor
     * @param xnot starting x position
     * @param ynot starting y position
     * */

    public skystoneFieldMap (int xnot, int ynot) {
        //ensuring coordinates are within field bounds
        if (xnot > 72)
            x = 72;
        else if (xnot < -72)
            x = -72;
        else
            x = xnot;

        if (ynot > 72)
            y = 72;
        else if (ynot < -72)
            y = -72;
        else
            y = ynot;

        //creating empty field
        for (int i = 0; i < field.length; i++) {
            for (int j = 0; j < field[i].length; j++) {
                field[i][j] = 0;
            }
        }

        //filling in foundations
        for (int i = 4; i < 39; i++) {
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
        for (int i = 95; i < field.length; i++){
            for (int j = 47; j < 52; j++){
                field[i][j] = 3;
            }

            for (int j = 92; j < 97; j++){
                field[i][j] = 3;
            }

        }

        //checking coordinate legality
        if (field[-y+72][x+72] != 0){
            throw new IllegalArgumentException("Coordinates must be in empty field");
        }
        else {
            field[72-y][x+72] = 1;
        }
    }

    public String toString(){
        return x + ", " + y;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public void setX(int newX){
        if (newX > 72)
            x = 72;
        else if (newX < -72)
            x = -72;
        else
            x = newX;
    }

    public void setY(int newY) {
        int tempY = newY;
        if (newY > 72)
            tempY = 72;
        else if (newY < -72)
            tempY = -72;
        else if (field[-tempY+72][x+72] != 0)
            throw new IllegalArgumentException("Coordinates must be in empty field");
        else {
            field[-y+72][x+72] = 0;
            y = tempY;
            field[-y+72][x+72] = 1 ;
        }
    }

}
