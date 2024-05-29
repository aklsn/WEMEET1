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

    public float maxSpeed = 100f; // 자동차의 최대 속력
    public float accelerationRate = 10f; // 가속률
    public float decelerationRate = 5f; // 자연 감속률
    public float brakeForce = 20f; // 브레이크 힘
    public float maxSteeringAngle = 45f; // 최대 핸들 각도
    public float steeringSensitivity = 0.1f; // 핸들 감도

    public GameObject steeringWheel; // 핸들 GameObject

    private Rigidbody rb;
    private float currentSpeed = 0f; // 현재 속도
    private float currentSteeringAngle = 0f; // 현재 핸들 각도
    private ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.9f, 0); // 자동차의 안정성을 높이기 위해 무게 중심 설정

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
                    // 악셀레이터, 브레이크, 핸들 포텐셜미터 값
                    float acceleratorValue = float.Parse(sensorValues[0]);
                    float brakeValue = float.Parse(sensorValues[1]);
                    float steeringValue = float.Parse(sensorValues[2]);

                    // 가속 및 감속 처리
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
                // 형식 변환 에러 처리
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
                    // 시간 초과 에러 처리
                }
            }
            await Task.Delay(10); // CPU 사용량을 줄이기 위해 잠시 대기
        }
    }

    void ApplyAcceleration(float acceleration)
    {
        // 가속도 적용
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
        // 브레이크 적용
        if (brake > 0)
        {
            currentSpeed -= brake * brakeForce * (Time.deltaTime * 100);
            currentSpeed = Mathf.Clamp(currentSpeed, 0f, maxSpeed);
        }

        rb.velocity = transform.forward * currentSpeed;
    }

    void ApplySteering(float steering)
    {
        // 핸들 각도 적용
        currentSteeringAngle = steering * maxSteeringAngle;

        // 핸들 GameObject 회전
        if (steeringWheel != null)
        {
            steeringWheel.transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -currentSteeringAngle));
        }
    }

    void FixedUpdate()
    {
        // 자동차의 회전을 적용
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
