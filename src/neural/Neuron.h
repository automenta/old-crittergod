/*
 * Neuron.h
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#ifndef NEURON_H_
#define NEURON_H_



class AbstractNeuron {

public:
	virtual float getOutput() {
		return 0;
	}

};

class Synapse {
public:
	AbstractNeuron* inputNeuron; // InputNeuron's Output Pointer

	float weight; // its synaptic weight -1.0f <-> +1.0f

	//float dendriteBranch; // dendritic branch synapse is located on

	void print() {
		cout << "       Synapse(input?=" << inputNeuron << ", weight=" << weight << ")\n" ;
	}
	Synapse(AbstractNeuron* _inputNeuron, float _synapticWeight /*, float _dendriteBranch*/ ) {
		inputNeuron = _inputNeuron;
		weight = _synapticWeight;
		//dendriteBranch = _dendriteBranch;
	}

	virtual float getInput() {
		return inputNeuron->getOutput();
	}

};

class InNeuron: public AbstractNeuron {

private:
	float input; //sensed input

public:
	InNeuron() {
		AbstractNeuron();
		input = 0;
	}

	float getInput() { return input; }

	void setInput(float i) {
		input = i;
	}
	virtual float getOutput() {
		//cout << "inNeuron gotOutput: " << input << "\n";
		return input;
	}
};

class OutNeuron: public AbstractNeuron {
	float potential;
	float stimulationFactor; //sensitivity to input stimulus.  defines the discreteness or detail level of its output spectrum

public:
	OutNeuron() {
		AbstractNeuron();
		stimulationFactor = 0.05;
	}

	void reset() {
		potential = 0;
	}

	void stimulate(float p) {
		potential +=p * stimulationFactor;
		if (potential > 1.0) potential = 1.0;
		if (potential < -1.0) potential = -1.0;
	}

	virtual float getOutput() {
		return potential;
	}
};

class SynapseBuilder {
public:
    // determines if id referes to an interneuron or sensorneuron
    bool isSensorNeuron;

    // id of neuron which axon connects to this synapse
    //InterNeuron neuron;

    //int realneuronID;
    int neurontargetlayer;

    // dendridic weight according
    //int dendriteBranches;

    // its "weight"
    float weight;

    AbstractNeuron* inNeuron;

    SynapseBuilder() {
        isSensorNeuron = false;
        neurontargetlayer = 0;
        //dendriteBranches = 1;
        weight = 0.0F;
    }

};

/** inter-neuron */
class Neuron: public OutNeuron {
public:
	OutNeuron* target;

	vector<Synapse*> synapses;
	vector<SynapseBuilder*> synapseBuilders;

	float output, nextOutput;
	int maxSynapses;
	bool isInhibitory;
	float firingThreshold;
	//float dendridicBranches;
	float potential;
	float potentialDecay;
	bool isPlastic;
	float plasticityStrengthen;
	float plasticityWeaken;
	float maxSynapticWeightMagnitude; //was originally =5.0? why

	void print() {
		cout << "  Neuron" << '\n';
		cout << "    output? " << (target!=NULL) << '\n';
		cout << "    inhibitory? " << isInhibitory << '\n';
		cout << "    plastic? " << isPlastic << '\n';
		cout << "    maxSynapticWeightMagnitude? " << maxSynapticWeightMagnitude << '\n';
		cout << "    plasticity strengthen=" << plasticityStrengthen << ", weaken=" << plasticityWeaken << '\n';
		cout << "    potential decay " << potentialDecay << '\n';
		cout << "    firing threshold " << firingThreshold << '\n';

		for (unsigned i = 0; i < synapses.size(); i++) {
			Synapse* s = synapses[i];
			s->print();
		}
	}

	Neuron() {
		isInhibitory = false;


		isPlastic = false;		// plasticity up & down

		potential = 0.0;
		output = nextOutput = 0;

		target = NULL;
	}

	void forward(float dt) {
		//TODO handle 'dt' appropriately

		// potential decay
		potential *= potentialDecay;

		// make every connection do it's influence on the neuron's total potential

		for (unsigned i = 0; i < synapses.size(); i++) {
			Synapse* s = synapses[i];

			// lower synaptic weights
			if (isPlastic) {
				s->weight *= plasticityWeaken;
			}

			potential += s->weight * s->getInput() /** s->dendriteBranch * */;
			//cout << "Synapse " << s << " " << s->getInput() << " :: pot=" << potential << "\n";
		}

		if (isInhibitory) {
			forwardInhibitory();
		} else {
			forwardExhibitory();
		}
	}

	void forwardInhibitory() {
		// do we spike/fire
		if (potential <= -1.0f * firingThreshold) {
			// reset neural potential
			potential = 0.0f;

			// fire the neuron
			nextOutput = -1;

			// PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
			if (isPlastic) {
				for (unsigned i = 0; i < synapses.size(); i++) {
					Synapse* s = synapses[i];
					double o = s->getInput();

					// if synapse fired, strengthen the weight
					if ((o < 0.0f && s->weight > 0.0f) || (o > 0.0f && s->weight < 0.0f)) {
						s->weight *= plasticityStrengthen;
					}

					// clamp weight
					clampWeight(s);
				}
			}
		} // don't fire the neuron
		else {
			nextOutput = 0;
			// reset potential if < 0
			if (potential > 0.0f) {
				potential = 0.0f;
			}
		}
	}

	void forwardExhibitory() {
		// do we spike/fire
		if (potential >= firingThreshold) {
			// reset neural potential
			potential = 0.0f;

			// fire the neuron
			nextOutput = 1;

			// PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
			if (isPlastic) {
				for (unsigned i = 0; i < synapses.size(); i++) {
					Synapse* s = synapses[i];
					double o = s->getInput();

					// if synapse fired, strengthen the weight
					if ((o > 0.0f && s->weight > 0.0f) || (o < 0.0f && s->weight < 0.0f)) {
						s->weight *= plasticityStrengthen;
					}

					// if weight > max back to max
					clampWeight(s);
				}
			}
		} // don't fire the neuron
		else {
			nextOutput = 0;
			// reset potential if < 0
			if (potential < 0.0f) {
				potential = 0.0f;
			}
		}
	}

	void clampWeight(Synapse* s) {
		s->weight = fmax(fmin(s->weight, maxSynapticWeightMagnitude), -maxSynapticWeightMagnitude);
	}


};


class NeuronBuilder {
public:

	// inhibitory neuron by flag
    bool isInhibitory;
    // Consistent Synapses flag
    bool hasConsistentSynapses;
    // inhibitory synapses flag
    bool hasInhibitorySynapses;
    // neuron firing potential
    double firingThreshold;
    // dendridic branches
    double maxDendridicBranches;
    // motor neuron ability (excititatory only) flag
    //boolean isMotor; //isMotor if motor!=null
    // function
    OutNeuron* out;
    // synaptic plasticity by flag
    bool isPlastic;
    // factors
    float plasticityStrengthen;
    float plasticityWeaken;

    //vector<SynapseBuilder*> synapseBuilders;


    Neuron* newNeuron(int maxSynapses) {
        Neuron* ni = new Neuron();

        ni->maxSynapses = maxSynapses;

        ni->isInhibitory = isInhibitory;
        ni->firingThreshold = firingThreshold;

        //ni->dendridicBranches = maxDendridicBranches;

        ni->target = out;

        ni->isPlastic = isPlastic;

		ni->plasticityStrengthen = plasticityStrengthen;
		ni->plasticityWeaken = plasticityWeaken;

		//maximum that a synapse can multiply a signal. 1.0 = conserved
		ni->maxSynapticWeightMagnitude = 5.0;

		ni->potentialDecay = 0.995;

        //ni->synapseBuilders = synapseBuilders;

        return ni;
    }

};


#endif /* NEURON_H_ */
