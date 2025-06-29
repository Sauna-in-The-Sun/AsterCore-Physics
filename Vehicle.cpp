class Wheel {
    PacejkaTireModel tireModel;
    void update(float slipAngle) {
        Vector3 force = tireModel.calculateForce(slipAngle);
        
    }
};