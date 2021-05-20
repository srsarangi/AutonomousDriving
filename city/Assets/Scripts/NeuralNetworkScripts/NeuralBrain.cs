using System;
using System.Collections.Generic;
using NeuralNetwork.NetworkModels;
using UnityEngine;
using System.IO;
using System.Threading;

public class NeuralBrain : MonoBehaviour
{
	public int numSamples;
	public int numInputs, numOutputs;
	public int[] hiddenLayer;
	public TrainingType trainType;
	public int epochs;
	public double minimumError;
	public double learningRate, momentum;
	public double normalizingValue;

	private NeuralNetwork.NetworkModels.Network network;
	private int[] hiddenLayers;
	private List<DataSet> dataSets;
	private DataSet tempDataSet;

	private String readPath = "G:\\Work\\Unity\\Unity Projects\\carSimulator\\Assets\\CAS_trainingData\\TrainingData.txt";
	private StreamReader reader;
	private double[] values, targets;
	private int lineCount;

	public bool trained{ get; private set; }

	private Thread trainingThread;

	public double[] Predict (double[] values)
	{
		if (trained) {
			double[] results = network.Compute (values);
			//Debug.Log (results [1]);
			return (new double[]{ (results [0] * 60 - 30) });//, results [1] });
		} else
			return new double[]{ 0 };//, 0 };
	}

	void TrainNetwork ()
	{
		while (true) {
			if (network == null) {
				network = new NeuralNetwork.NetworkModels.Network (numInputs, hiddenLayer, numOutputs);
				network.LearnRate = learningRate;
				network.Momentum = momentum;
			}

			Debug.Log ("Network is training, this can take a while...");

			if (trainType == TrainingType.Epoch)
				network.Train (dataSets, epochs);
			else
				network.Train (dataSets, minimumError);

			Debug.Log ("Training complete!");
			trained = true;
			break;
		}
	}

	public bool Initiallize (int dataSetType)
	{
		reader = new StreamReader (readPath);

		int index = 0;
		lineCount = 0;

		dataSets = new List<DataSet> ();

		while (lineCount < numSamples) {

			String line = reader.ReadLine ();
			String[] readValues = line.Split (',');

			targets = new double[numOutputs];
			targets [0] = (double.Parse (readValues [0]) + 30) / 60;
			//targets [1] = (double.Parse (readValues [1]));

			double temp = 0;
			values = new double[numInputs];
			for (int i = 0; i < numInputs; i++) {
				temp = double.Parse (readValues [i + 1]);
				values [i] = temp * normalizingValue;
			}
				
			if (targets [0] != 0)
				dataSets.Add (new DataSet (values, targets));

			lineCount++;
		}
		trainingThread = new Thread (TrainNetwork);
		reader.Close ();

		Debug.Log ("Initiallization complete.");

		return true;
	}

	public void StartTrainingThread ()
	{
		trainingThread.Start ();
	}

	public void Reset ()
	{
		network = null;
		trained = false;

		if (dataSets != null)
			dataSets.Clear ();

		lineCount = 0;

		if (reader != null)
			reader.Close ();

		if (trainingThread != null && trainingThread.IsAlive) {
			trainingThread.Abort ();
			trainingThread = null;
		}
	}
}