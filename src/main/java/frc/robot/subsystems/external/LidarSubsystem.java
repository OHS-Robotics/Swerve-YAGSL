package frc.robot.subsystems.external;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.hal.I2CJNI;


// this code is based off of https://github.com/Paradox2102/Artemis/blob/master/src/team2102/lib/LIDARLite.java
// https://www.chiefdelphi.com/t/lidar-lite-v3-distance-measurement/162756/3

public class LidarSubsystem extends SubsystemBase {
    private static final byte k_deviceAddress = 0x62;

	private final byte m_port;

	private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);
    
    public LidarSubsystem(Port port) {
        m_port = (byte) port.value;
		I2CJNI.i2CInitialize(m_port);
    }

    public void startMeasuring() {
		writeRegister(0x04, 0x08 | 32); // default plus bit 5
		writeRegister(0x11, 0xff);
		writeRegister(0x00, 0x04);
	}

	public void stopMeasuring() {
		writeRegister(0x11, 0x00);
	}

	public int getDistance() {
		return readShort(0x8f);
	}

	private int writeRegister(int address, int value) {
		m_buffer.put(0, (byte) address);
		m_buffer.put(1, (byte) value);

		return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
	}

	private short readShort(int address) {
		m_buffer.put(0, (byte) address);
		I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
		I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
		return m_buffer.getShort(0);	
	}

	public double pidGet() {
		return getDistance();
	}
}
