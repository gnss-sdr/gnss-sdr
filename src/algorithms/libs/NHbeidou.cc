void decodeBCHBeidou(std::list<bool> *firstBranch_encoded, std::list<int> *output)
{
        bool input[15];
        std::copy(firstBranch_encoded.begin(),firstBranch_encoded.end(),input);
	std::array<bool, 4> D_register = {0,0,0,0};
	std::array<bool, 15> stage_buffer;
	std::array<bool, 15> ROM_list_circuit;
	for (i = 0; i < 15; i++)
	    {
		D_register = {inputBit[i] ^ D_register[3], D_register[0] ^ D_register[3], D_register[1], D_register[2]};
		stage_buffer[i] = inputBit[i];
	    }

	if(D_register == {0,0,0,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {0,0,0,1}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};

	} else if(D_register == {0,0,1,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0};

	} else if(D_register == {0,0,1,1}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0};

	} else if(D_register == {0,1,0,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0};

	} else if(D_register == {0,1,0,1}) {

	    ROM_list_circuit = {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0};

	} else if(D_register == {0,1,1,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0};

	} else if(D_register == {0,1,1,1}) {

	    ROM_list_circuit = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {1,0,0,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0};

	} else if(D_register == {1,0,0,1}) {

	    ROM_list_circuit = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {1,0,1,0}) {

	    ROM_list_circuit = {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {1,0,1,1}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0};

	} else if(D_register == {1,1,0,0}) {

	    ROM_list_circuit = {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0};

	} else if(D_register == {1,1,0,1}) {

	    ROM_list_circuit = {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {1,1,1,0}) {

	    ROM_list_circuit = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0};

	} else if(D_register == {1,1,1,1}) {

	    ROM_list_circuit = {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0};

	}

	for (i = 0; i < 15; i++)
	    {
		if(stage_buffer[i] ^ ROM_list_circuit[i])
		    {
			output.push_back(1);
		    }
		else
		    {
			output.push_back(-1);
		    }
	    }

}

void remove_NH_Beidou(bool *input, bool *output)
{
    int *code_NH_Beidou = {-1,-1,-1,-1,-1,+1,-1,-1,+1,+1,-1,+1,-1,+1,-1,-1,+1,+1,+1,-1};
    int corr_NH = 0;
    int correlation = 0;

    for (int i = 0; i < 20; i++)
	{
	    if ((code_NH_Beidou[i] * input[i]) > 0.0)
		{
		    corr_NH += 1;
		}
	    else
		{
		    corr_NH -= 1;
		}
	}
    if (abs(corr_NH) == 20)
	{
	    correlation = 1;
	    if (corr_NH > 0)
		{
		    output = 1;
		}
	    else
		{
		    output = 0;
		}
	}
}

void process_TRK_output_Beidou(bool *input, int n_input_bits, int *output)
{
    bool buffer_NH[15];
    bool new_bit;
    std::list<bool> firstBranch_encoded;
    std::list<int> firstBranch_decoded;
    std::list<bool> secondBranch_encoded;
    std::list<int> secondBranch_decoded;
    std::list<int> output_list;

    for (int i = 0; i < n_input_bits/15; i++)
        {
            for (int j = 0; j < 15; i++)
                {
                    buffer_NH[j] = input[i + j];
                }
            remove_NH_Beidou(buffer_NH, &new_bit)

            if ( i % 2 == 0 )
                {
                    firstBranch_encoded.push_back(new_bit);
                }
            else
                {
                    secondBranch_encoded.push_back(new_bit);
                }
            if (firstBranch_encoded.size() == 15)
                {                    
                    decodeBCHBeidou(&firstBranch_encoded, &firstBranch_decoded);
                    firstBranch_encoded.clear();
                }
            if (secondBranch_encoded.size() == 15)
                {
                    decodeBCHBeidou(&secondBranch_encoded, &secondBranch_decoded);
                    secondBranch_encoded.clear();
                }
            if (firstBranch_decoded.size() > 10)
                {
                    for (i = 0; i < 11; i++)
	                {
            	            output_list.push_back(firstBranch_decoded.front());
                            firstBranch_decoded.pop_front();
           	        }
                }
            if (secondBranch_decoded.size() > 10)
                {
                    for (i = 0; i < 11; i++)
                        {
                            output_list.push_back(secondBranch_decoded.front());
                            secondBranch_decoded.pop_front();
                        }
                }
        }
    std::copy(output_list.begin(),output_list.end(),output);
}
