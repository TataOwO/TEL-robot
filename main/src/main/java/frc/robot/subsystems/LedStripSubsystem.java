
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedStripConstants;

public class LedStripSubsystem extends SubsystemBase {
    private final AddressableLED m_light;
    private final AddressableLEDBuffer m_buffer;
    private final int m_length;
    private final TransporterSubsystem m_transport;
    private final ColorList color_list;

    public LedStripSubsystem(TransporterSubsystem transport) {
        m_transport = transport;

        m_length = LedStripConstants.LED_COUNT;

        color_list = new ColorList(m_length);

        m_buffer = new AddressableLEDBuffer(m_length);

        m_light = new AddressableLED(LedStripConstants.PWM_PIN);
        m_light.setLength(m_length);
        m_light.setData(m_buffer);

        m_light.start();
    }

    @Override
    public void periodic() {
        color_list.update();

        if (m_transport.getButton()) {
            for (int i=0; i<m_length; ++i) {
                m_buffer.setHSV(i, 120, 255, 128);
            }
        } else {
            for (int i=0; i<m_length; ++i) {
                float[] val = color_list.get(i);
                int[] hsv = new int[] {(int)val[0]*255, (int)val[1]*255, (int)val[2]*255};
                m_buffer.setHSV(i, hsv[0], hsv[1], hsv[2]);
            }
        }

        m_light.setData(m_buffer);
    }

    private class ColorList {
        private final List<float[]> m_color_list = new ArrayList<>();
        private final int length;
        private int tick = 0;
        
        public ColorList(int length) {
            for (int i=0; i<length; ++i) {
                m_color_list.add(new float[] {0,0,0});
            }

            this.length = length;
        }

        public float[] get(int index) {
            return m_color_list.get(index);
        }

        public void update() {
            ++tick;
            
            setRGB(.1f, tick);
            
            int spark_length = 3;
            int offset = spark_length*2;
            int spark_index = (int)(0.02*tick*length);
            float[] CYAN = new float[] {.5f, 1f, 1f};
            
            setSparkline(CYAN, spark_index+offset*0, spark_length);
            setSparkline(CYAN, spark_index+offset*1, spark_length);
            setSparkline(CYAN, spark_index+offset*2, spark_length);
        }

        private void setSparkline(float[] color, int index, int length) {
            index = index % (this.length + length) - length;
            
            for (int i=0; i<length; i++) {
                int current_index = i + index;
                
                if (current_index<0) continue;
                if (current_index>=this.length) continue;
                
                float[] org_color = get(current_index);
                
                float modifier = (float)i/length;
                modifier = (float) Math.sin(modifier * Math.PI / 2);
                
                float[] new_color = new float[] {
                    color[0]*modifier + org_color[0]*(1-modifier),
                    color[1]*modifier + org_color[1]*(1-modifier),
                    color[2]*modifier + org_color[2]*(1-modifier)
                };
                
                m_color_list.set(current_index, new_color);
            }
        }
        
        private void setRGB(float brightness, int index) {
            for (int i=0; i<length; ++i) {
                float hue = (((float)i+index)%this.length) / this.length;
                float[] waveColor = new float[] {hue, 1f, brightness};
                
                m_color_list.set(i, waveColor);
            }
        }
    }
}
