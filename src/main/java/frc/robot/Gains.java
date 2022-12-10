package frc.robot;

public class Gains {
    public double P;
    public double I;
    public double D;
    public double F;
    public int IZONE;
    public double PEAK_OUTPUT;
	
	public Gains(double _P, double _I, double _D, double _F, int _IZONE, double _PEAK_OUTPUT){
		P = _P;
		I = _I;
		D = _D;
		F = _F;
		IZONE = _IZONE;
		PEAK_OUTPUT = _PEAK_OUTPUT;
	}
}
