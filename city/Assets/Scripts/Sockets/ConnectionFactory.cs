using System;
using System.Net;
using System.Text;
using System.Net.Sockets;
using System.Collections.Generic;

namespace InferenceCommunication
{
    public static class ConnectionPool
    {
        /// <summary>
        /// Queue of available socket connections.
        /// </summary>
        private static Queue<CustomSocket> availableSockets = null;
        /// <summary>
        /// host IP Address
        /// </summary>
        private static string hostIP = string.Empty;
        /// <summary>
        /// host Port
        /// </summary>
        private static int hostPort = 0;
        /// <summary>
        /// Initial number of connections
        /// </summary>
        private static int POOL_MIN_SIZE = 5;
        /// <summary>
        /// The maximum size of the connection pool.
        /// </summary>
        private static int POOL_MAX_SIZE = 20;
        /// <summary>
        /// Created host Connection counter 
        /// </summary>
        private static int SocketCounter = 0;

        public static bool Initialized = false;

        static ConnectionPool()
        {

        }

        /// <summary>
        /// Initialize host Connection pool
        /// </summary>
        /// <param name="hostIP">host IP Address</param>
        /// <param name="hostPort">host Port</param>
        /// <param name="minConnections">Initial number of connections</param>
        /// <param name="maxConnections">The maximum size of the connection pool</param>
        public static void InitializeConnectionPool(string hostIPAddress, int hostPortNumber, int minConnections, int maxConnections)
        {
            POOL_MAX_SIZE = maxConnections;
            POOL_MIN_SIZE = minConnections;
            hostIP = hostIPAddress;
            hostPort = hostPortNumber;
            availableSockets = new Queue<CustomSocket>();

            for (int i = 0; i < minConnections; i++)
            {
                CustomSocket cachedSocket = OpenSocket();
                PutSocket(cachedSocket);
            }

            Initialized = true;

            System.Diagnostics.Trace.WriteLine("Connection Pool is initialized with Max Number of " +
                    POOL_MAX_SIZE.ToString() + " And Min number of " + availableSockets.Count.ToString());
        }

        /// <summary>
        /// Get an open socket from the connection pool.
        /// </summary>
        /// <returns>Socket returned from the pool or new socket opened. </returns>
        public static CustomSocket GetSocket()
        {
            if (ConnectionPool.availableSockets.Count > 0)
            {
                lock (availableSockets)
                {
                    CustomSocket socket = null;
                    while (ConnectionPool.availableSockets.Count > 0)
                    {
                        socket = ConnectionPool.availableSockets.Dequeue();

                        if (socket.Connected)
                        {
                            System.Diagnostics.Trace.WriteLine("Socket Dequeued -> Pool size: " +
                                ConnectionPool.availableSockets.Count.ToString());

                            return socket;
                        }
                        else
                        {
                            socket.Close();
                            System.Threading.Interlocked.Decrement(ref SocketCounter);
                            System.Diagnostics.Trace.WriteLine("GetSocket -- Close -- Count: " + SocketCounter.ToString());
                        }
                    }
                }
            }

            return ConnectionPool.OpenSocket();
        }

        /// <summary>
        /// Return the given socket back to the socket pool.
        /// </summary>
        /// <param name="socket">Socket connection to return.</param>
        public static void PutSocket(CustomSocket socket)
        {
            lock (availableSockets)
            {
                TimeSpan socketLifeTime = DateTime.Now.Subtract(socket.TimeCreated);

                if (ConnectionPool.availableSockets.Count < ConnectionPool.POOL_MAX_SIZE &&
                    socketLifeTime.Minutes < 2)// Configuration Value
                {
                    if (socket != null)
                    {
                        if (socket.Connected)
                        {
                            ConnectionPool.availableSockets.Enqueue(socket);

                            System.Diagnostics.Trace.WriteLine("Socket Queued -> Pool size: " +
                                ConnectionPool.availableSockets.Count.ToString());
                        }
                        else
                        {
                            socket.Close();
                        }
                    }
                }
                else
                {
                    socket.Close();
                    System.Diagnostics.Trace.WriteLine("PutSocket - Socket is forced to closed -> Pool size: " +
                                    ConnectionPool.availableSockets.Count.ToString());
                }
            }
        }

        /// <summary>
        /// Open a new socket connection.
        /// </summary>
        /// <returns>Newly opened socket connection.</returns>
        private static CustomSocket OpenSocket()
        {
            if (SocketCounter < POOL_MAX_SIZE)
            {
                System.Threading.Interlocked.Increment(ref SocketCounter);
                System.Diagnostics.Trace.WriteLine("Created host Connections count: " + SocketCounter.ToString());
                return new CustomSocket(hostIP, hostPort);
            }

            throw new Exception("Connection Pool reached its limit");
        }

        /// <summary>
        /// Populate host socket exception on sending or receiveing
        /// </summary>
        public static void PopulateSocketError()
        {
            System.Threading.Interlocked.Decrement(ref SocketCounter);
            System.Diagnostics.Trace.WriteLine("Populate Socket Error host Connections count: " + SocketCounter.ToString());
        }
    }
}

public class CustomSocket : TcpClient
{
    private DateTime _TimeCreated;

    public DateTime TimeCreated
    {
        get { return _TimeCreated; }
        set { _TimeCreated = value; }
    }

    public CustomSocket(string host, int port)
        : base(host, port)
    {
        _TimeCreated = DateTime.Now;
    }
}


