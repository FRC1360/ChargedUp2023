package frc.robot.autos.basic;


public enum Direction {
    LEFT, RIGHT, FRONT_CENTER, BACK; 

    public Direction getOpposite() { 
        
        if (this == Direction.FRONT_CENTER) return Direction.BACK; 
        else if (this == Direction.BACK) return Direction.FRONT_CENTER; 
        else if (this == Direction.LEFT) return Direction.RIGHT; 
        return Direction.LEFT; 
    }
}
