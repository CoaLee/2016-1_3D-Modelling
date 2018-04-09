using UnityEngine;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System;
using System.Linq;
using System.Text;

/// <summary>
/// Behaviour for mouse movement.
/// the message format from the device was modified, so now the message has following ascii string format
///  [position_x];[position_y];[position_z]#[crc32_hash_in_hex]\n
/// note that position_* has the format of decimal float string, and each of them may contain spaces in front of/back of them.
/// </summary>
public class Lucidraw : MonoBehaviour {
    /// <summary>
    /// sensitivity of the device.
    /// </summary>
    public float translateSensitivity = 0;
    public float rotateSensitivity = 0;

    /// <summary>
    /// name of the port
    /// </summary>
    public string portName = "COM4";

    private SerialPort sp;

    void Start() {
        sp = new SerialPort(portName, 115200);
        // be aware of the unit of line below is mllisecond(ms)
        sp.ReadTimeout = 1000;
        sp.Open();
    }

    /// <summary>
    /// Update logic for this behaviour.
    /// the logic is as following
    /// 1. send query string("s")
    /// 2. read for one line in the serial.
    /// 3. split that line into message and hash part(divide by '#')
    /// 3-1. if the line has not divided into exact two part by '#', than it's FORMAT_FAILED case.
    /// 4. check if message and hash has the right format
    /// 4-1. if it does not match, than it's also FORMAT_FAILED case.
    /// 5. check for crc hash.
    /// 6. parse position vector in message, set transform to that.
    /// </summary>
    void Update() {
        string floatExpr = "(\\s*[-+]?(?:[0-9]*\\.[0-9]+|[0-9]+)\\s*)";
        string hexExpr = "[A-Fa-f0-9]+";
        Regex hashExpr = new Regex(String.Format("^{0}$", hexExpr));
        Regex msgExpr = new Regex(String.Format("^(?:{0};){{3}}(?:{1};){{4}}$", floatExpr, hexExpr));

        if (sp.IsOpen)
        {
            try
            {
                sp.DiscardInBuffer();
                sp.Write("s");

                string response = sp.ReadLine();
                string[] messageAndHash = response.Split('#');

                if (messageAndHash.Length == 2)
                {
                    string message = messageAndHash[0];
                    string hash = messageAndHash[1];

                    if (hashExpr.IsMatch(hash) && msgExpr.IsMatch(message))
                    {
                        if (checkForCrc32(message, hash))
                        {
                            string[] svector = message.Split(new char[] { ';' }, StringSplitOptions.RemoveEmptyEntries);

                            if (svector.Length == 7)
                            {
                                var pvector = (from v in svector.Take(3) select float.Parse(v.Trim())).ToList();
                                var rvector = (from v in svector.Skip(3).Take(4) select Convert.ToSByte(v.Trim(), 16)).ToList();
                                transform.position = new Vector3(pvector[0], pvector[2], pvector[1]) * translateSensitivity;
                                transform.Rotate(new Vector3(-rvector[2], rvector[0], rvector[3]) * rotateSensitivity / 200, Space.World);
                            }
                        }
                    }
                }
                else
                {
                    // FORMAT_FAILED case. does nothing
                    Debug.Log(String.Format("[WARNING] FORMAT_FAILED at the message \"{0}\"", response));
                }
            }
            catch (System.Exception err)
            {
                Debug.Log(err);
            }
        }
    }

    /// <summary>
    /// check the message for crc32 hash
    /// </summary>
    /// <param name="message"></param>
    /// <param name="hashString"></param>
    /// <returns>if the message matches for hash</returns>
    private bool checkForCrc32(string message, string hashString)
    {
        byte[] messageByte = Encoding.ASCII.GetBytes(message);
        UInt32 hashFromMessage = Convert.ToUInt32(hashString, 16);
        UInt32 hashFromCalculation = __crc32__.calculate(0xFFFFFFFU, messageByte, 0, messageByte.Length);

        if (hashFromCalculation == hashFromMessage)
        {
            return true;
        }
        else
        {
            Debug.Log(String.Format(
                "[WARNING] CRC_FAILED({0:X} != {1:X}) at the message {2}",
                hashFromCalculation,
                hashFromMessage,
                message + "#" + hashString
            ));
            return false;
        }
    }
}