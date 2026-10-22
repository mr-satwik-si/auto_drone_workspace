#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

class AlertManagerNode(Node):
    def __init__(self):
        super().__init__('alert_manager_node')

        self.declare_parameter('smtp_host', 'smtp.gmail.com')
        self.declare_parameter('smtp_port', 587)
        self.declare_parameter('smtp_user', '')
        self.declare_parameter('smtp_pass', '')
        self.declare_parameter('alert_email', '')

        self.smtp_host   = self.get_parameter('smtp_host').value
        self.smtp_port   = self.get_parameter('smtp_port').value
        self.smtp_user   = self.get_parameter('smtp_user').value
        self.smtp_pass   = self.get_parameter('smtp_pass').value
        self.alert_email = self.get_parameter('alert_email').value

        self.create_subscription(String, '/alerts', self.alert_callback, 10)
        self.get_logger().info('AlertManagerNode started — monitoring FIRE and PERSON alerts only')

    def alert_callback(self, msg):
        data = msg.data

        # Only process FIRE and PERSON alerts — ignore temperature and sound
        if 'FIRE' in data:
            subject = '🔥 [DRONE ALERT] FIRE DETECTED in Room!'
            body = self._build_email_body(
                alert_type='FIRE',
                icon='🔥',
                color='#ff3300',
                title='Fire Detected!',
                detail=data,
                action='Immediately check the room and call emergency services if needed.'
            )
            self.get_logger().warn(f'FIRE alert — sending email: {data}')
            self._send_email(subject, body)

        elif 'PERSON' in data:
            subject = '🧍 [DRONE ALERT] PERSON DETECTED in Room!'
            body = self._build_email_body(
                alert_type='PERSON',
                icon='🧍',
                color='#00aa44',
                title='Person Detected!',
                detail=data,
                action='A person has been detected in the monitored room.'
            )
            self.get_logger().warn(f'PERSON alert — sending email: {data}')
            self._send_email(subject, body)

        else:
            # Silently ignore temperature and sound alerts
            pass

    def _build_email_body(self, alert_type, icon, color, title, detail, action):
        return f"""
        <html>
        <body style="font-family: Arial, sans-serif; background: #0a0a1a; color: #e0e0e0; padding: 20px;">
          <div style="max-width: 600px; margin: auto; background: #12122a;
                      border-radius: 12px; padding: 30px;
                      border: 2px solid {color};">

            <h1 style="color: {color}; font-size: 28px; margin-bottom: 8px;">
              {icon} {title}
            </h1>

            <p style="color: #aaa; font-size: 14px; margin-bottom: 24px;">
              Drone Room Monitor — Automatic Alert
            </p>

            <div style="background: #1a1a3a; border-radius: 8px;
                        padding: 16px; margin-bottom: 20px;
                        border-left: 4px solid {color};">
              <p style="margin: 0; font-size: 14px; color: #e0e0e0;">
                <strong>Alert:</strong> {detail}
              </p>
            </div>

            <div style="background: #1a1a3a; border-radius: 8px;
                        padding: 16px; margin-bottom: 20px;">
              <p style="margin: 0; font-size: 14px; color: #e0e0e0;">
                <strong>Action Required:</strong> {action}
              </p>
            </div>

            <p style="color: #555; font-size: 12px; margin-top: 24px;">
              This alert was generated automatically by your Drone Room
              Monitoring System.
            </p>
          </div>
        </body>
        </html>
        """

    def _send_email(self, subject, body):
        if not self.smtp_user or not self.alert_email:
            self.get_logger().warn('Email not configured — skipping')
            return
        try:
            msg = MIMEMultipart('alternative')
            msg['Subject'] = subject
            msg['From']    = self.smtp_user
            msg['To']      = self.alert_email
            msg.attach(MIMEText(body, 'html'))

            with smtplib.SMTP(self.smtp_host, self.smtp_port) as server:
                server.starttls()
                server.login(self.smtp_user, self.smtp_pass)
                server.send_message(msg)

            self.get_logger().info(f'Email sent to {self.alert_email}')
        except Exception as e:
            self.get_logger().error(f'Email failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AlertManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()