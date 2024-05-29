using System;
using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;

public class CarController : MonoBehaviour
{
    SerialPort serialPort;
    string portName = "COM7";
    int baudRate = 9600;

    public float maxSpeed = 100f; // �ڵ����� �ִ� �ӷ�
    public float accelerationRate = 10f; // ���ӷ�
    public float decelerationRate = 5f; // �ڿ� ���ӷ�
    public float brakeForce = 20f; // �극��ũ ��
    public float maxSteeringAngle = 45f; // �ִ� �ڵ� ����
    public float steeringSensitivity = 0.1f; // �ڵ� ����

    public GameObject steeringWheel; // �ڵ� GameObject

    private Rigidbody rb;
    private float currentSpeed = 0f; // ���� �ӵ�
    private float currentSteeringAngle = 0f; // ���� �ڵ� ����
    private ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.9f, 0); // �ڵ����� �������� ���̱� ���� ���� �߽� ����

        serialPort = new SerialPort(portName, baudRate);
        serialPort.Open();
        cancellationTokenSource = new CancellationTokenSource();
        Task.Run(() => ReadSerialData(cancellationTokenSource.Token));
    }

    void Update()
    {
        while (dataQueue.TryDequeue(out string serialData))
        {
            try
            {
                string[] sensorValues = serialData.Split(',');

                if (sensorValues.Length >= 3)
                {
                    // �Ǽ�������, �극��ũ, �ڵ� ���ټȹ��� ��
                    float acceleratorValue = float.Parse(sensorValues[0]);
                    float brakeValue = float.Parse(sensorValues[1]);
                    float steeringValue = float.Parse(sensorValues[2]);

                    // ���� �� ���� ó��
                    float acceleration = Mathf.Clamp(acceleratorValue / 1023f, 0f, 1f);
                    float brake = Mathf.Clamp(brakeValue / 1023f, 0f, 1f);
                    float steering = Mathf.Clamp((steeringValue / 1023f * 2) - 1f, -1f, 1f);

                    ApplyAcceleration(acceleration);
                    ApplyBrake(brake);
                    ApplySteering(steering);

                    Debug.Log($"Accelerator: {acceleratorValue}, Brake: {brakeValue}, Steering: {steeringValue}, Speed: {currentSpeed}, Steering Angle: {currentSteeringAngle}");
                }
            }
            catch (FormatException)
            {
                // ���� ��ȯ ���� ó��
            }
        }
    }

    async Task ReadSerialData(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            if (serialPort.IsOpen)
            {
                try
                {
                    string serialData = await Task.Run(() => serialPort.ReadLine());
                    dataQueue.Enqueue(serialData);
                }
                catch (TimeoutException)
                {
                    // �ð� �ʰ� ���� ó��
                }
            }
            await Task.Delay(10); // CPU ��뷮�� ���̱� ���� ��� ���
        }
    }

    void ApplyAcceleration(float acceleration)
    {
        // ���ӵ� ����
        float targetSpeed = acceleration * maxSpeed;
        if (currentSpeed < targetSpeed)
        {
            currentSpeed += accelerationRate * (Time.deltaTime * 100);
        }
        currentSpeed = Mathf.Clamp(currentSpeed, 0f, maxSpeed);

        rb.velocity = transform.forward * currentSpeed;
    }

    void ApplyBrake(float brake)
    {
        // �극��ũ ����
        if (brake > 0)
        {
            currentSpeed -= brake * brakeForce * (Time.deltaTime * 100);
            currentSpeed = Mathf.Clamp(currentSpeed, 0f, maxSpeed);
        }

        rb.velocity = transform.forward * currentSpeed;
    }

    void ApplySteering(float steering)
    {
        // �ڵ� ���� ����
        currentSteeringAngle = steering * maxSteeringAngle;

        // �ڵ� GameObject ȸ��
        if (steeringWheel != null)
        {
            steeringWheel.transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -currentSteeringAngle));
        }
    }

    void FixedUpdate()
    {
        // �ڵ����� ȸ���� ����
        if (currentSpeed > 0)
        {
            float turn = currentSteeringAngle * steeringSensitivity * Time.fixedDeltaTime;
            Quaternion deltaRotation = Quaternion.Euler(0f, turn, 0f);
            rb.MoveRotation(rb.rotation * deltaRotation);
        }
    }

    void OnDestroy()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }

        if (cancellationTokenSource != null)
        {
            cancellationTokenSource.Cancel();
        }
    }
}
