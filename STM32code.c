#include "mbed.h"
#include "rtos.h"
#include "string.h"
#include <stdio.h>
#include <ctype.h>

#define SAMPLE_READY 1
#define SAMPLE_DONE 2

const int SIGBUF_SIZE = 120;
const int COMBUF_SIZE = 20;
const int ARGBUF_SIZE = 3;

volatile bool sampleenable = true;
//used to prevent sampling being renabled
//after a print/delete when it should
//have remained dissabled


AnalogIn A(A0);

Serial pc(USBTX, USBRX);

Ticker sampler;

DigitalOut Red(D7);
DigitalOut green(D5);

Thread userinp;
Thread sample;

int stringtoint(const char* str)
{
	int total = 0;
	
	for (int i=0; i<strlen(str); i++)
	{
		char c = str[i];
		
		//bound check characters
		if (c < 48| c > 57)
			return -1;//-1 indicates error
		
		total = total * 10;
		total += (c-48);
	}
	return total;
}

volatile float* sigbuffer = new float[SIGBUF_SIZE];
volatile int sigbufpos = 0;
volatile float average = 0.0f;
volatile float runningtotal = 0.0f;
volatile int recordcount = 0;

void sampletrigger()
{
	//reading from the analog input is not interupt
	//safe so we signal a thread instead
	sample.signal_set(SAMPLE_READY);
}

//thread
void sampleprocess()
{
	while (1)
	{
		Thread::signal_wait(SAMPLE_READY);
		
		userinp.signal_clr(SAMPLE_DONE);
		
		green = !green;
		runningtotal -= sigbuffer[sigbufpos];
		sigbuffer[sigbufpos] = A.read();
		runningtotal += sigbuffer[sigbufpos];
		sigbufpos = (sigbufpos + 1) % SIGBUF_SIZE;
		
		recordcount = (recordcount>=SIGBUF_SIZE) ? SIGBUF_SIZE : recordcount+1;
		
		//running total is used to avoid recomputing the
		//sum of all records each time one record is changed
		average = runningtotal / recordcount;
		
		//signal that we are done processing the new sample
		userinp.signal_set(SAMPLE_DONE);
		
		sample.signal_clr(SAMPLE_READY);
	}
}

void printdata(int amt)
{
	if (recordcount == 0)
	{
		pc.printf("No records in memory\r\n");
		return;
	}
	else
	{
		pc.printf("%i records in memory\r\n", recordcount);
		pc.printf("Average value: %f\r\n", average);
	}
	
	//entering critical section
	sampler.detach();
	
	//make sure that we wait untill the most
	//recent sample is done processing.
	//If sampling is dissabled then we
	//don't need to bother.
	if (sampleenable)
		Thread::signal_wait(SAMPLE_DONE);
	
	//clamp amt to the record count.
	//We can't print more records than we have.
	if (amt > recordcount)
		amt = recordcount;
	
	pc.printf("Printing %i records\r\n", amt);
	
	//if we assume that the buffer is full then
	//"sigbufpos" will point to the oldest record
	//in memory. if we then take "recordcount" into
	//account we can find the oldest record in
	//all cases.
	int offset = sigbufpos + ((SIGBUF_SIZE - recordcount) % SIGBUF_SIZE);
	
	for (int i=0; i<amt; i++)
	{
		int pos = (i+offset)%SIGBUF_SIZE;
		pc.printf("[%i]: %f\r\n", pos, sigbuffer[pos]);
	}
	
	if (sampleenable)
		sampler.attach(&sampletrigger, 0.1f);
}

void deletedata(int amt)
{
	if (recordcount == 0)
	{
		pc.printf("No records in memory\r\n");
		return;
	}
	else
	{
		pc.printf("%i records in memory\r\n", recordcount);
	}
	
	//entering critical section
	sampler.detach();
	
	//make sure that we wait untill the most
	//recent sample is done processing.
	//If sampling is dissabled then we
	//don't need to bother.
	if (sampleenable)
		Thread::signal_wait(SAMPLE_DONE);
	
	//clamp amt to the record count.
	//We can't delete more records than we have.
	if (amt > recordcount)
		amt = recordcount;
	
	pc.printf("Deleting %i records\r\n", amt);
	
	//if we assume that the buffer is full then
	//"sigbufpos" will point to the oldest record
	//in memory. if we then take "recordcount" into
	//account we can find the oldest record in
	//all cases.
	int offset = sigbufpos + ((SIGBUF_SIZE - recordcount) % SIGBUF_SIZE);
	
	for (int i=0; i<amt; i++)
	{
		runningtotal -= sigbuffer[(i+offset)%SIGBUF_SIZE];
		sigbuffer[(i+offset)%SIGBUF_SIZE] = 0.0f;
	}
	
	recordcount -= amt;
	
	//we have changed the running total so we should
	//update the average as well.
	average = runningtotal / recordcount;
	
	pc.printf("%i records deleted\r\n", amt);
	
	
	if (sampleenable)
		sampler.attach(&sampletrigger, 0.1f);
}


//Instead of reading input into one buffer
//and working out where the command ends
//and the argument begins we simply read
//into two different buffers
char* commandbuf = new char[COMBUF_SIZE];
int combufpos = 0;
char* argumentbuf = new char[ARGBUF_SIZE];
int argbufpos = 0;

char c;

//Thread
void userinput()
{
	reset:
	//goto is used to escape the input handling
	//loops in the event of an abort.
	//This should be less cumbersome and
	//error prone than setting a flag.
	
	//Sometimes garbage data remains in the buffers from
	//before a system reset.
	combufpos = argbufpos = 0;
	memset(commandbuf, 0x0, COMBUF_SIZE);
	memset(argumentbuf, 0x0, ARGBUF_SIZE);
	
	pc.printf("terminal is open\r\n");
	
	while(1)
	{
		
		//Handle input for command
		while(1)
		{
			c = pc.getc();
			
			if (c == 0x7F | c == 0xD)//backspace or enter aborts the whole procedure
			{
				pc.printf("\r\nCommand entry aborted\r\n");
				goto reset;
			}
			
			pc.putc(c);
			
			//space character indicates that commands entry
			//is done and we are ready for agrument entry.
			if (c == 0x20)//" "
				break;
			
			//detect command buffer overflows
			if (combufpos > COMBUF_SIZE)
			{
				pc.printf("\r\nComand max length exceeded\r\n");
				combufpos = 0;
				memset(commandbuf, 0x0, COMBUF_SIZE);
			}
			else
			{
				//add character to command buffer
				commandbuf[combufpos] = c;
				combufpos++;
			}
		}
		
		//handle input for argument
		while(1)
		{
			c = pc.getc();
			
			if (c == 0x7F | c == 0x20)//backspace or space aborts the whole procedure
			{
				pc.printf("\r\nCommand entry aborted\r\n");
				goto reset;
			}
			
			pc.putc(c);
			
			//enter character indicates that argument entry
			//is done and we are ready to act upon the command.
			if (c == 0xD)//"\r"
			{
				pc.putc(0xA);//"\n"
				break;
			}
			
			//detect argument buffer overflows
			if (argbufpos >= ARGBUF_SIZE)
			{
				pc.printf("\r\nArgument max length exceeded\r\n");
				
				//reprint the command so that the user
				//knows that they don't need to reenter it.
				pc.printf(commandbuf);
				pc.putc(0x20);
				
				argbufpos = 0;
				memset(argumentbuf, 0x0, ARGBUF_SIZE);
			}
			else
			{
				//add character to argument buffer
				argumentbuf[argbufpos] = c;
				argbufpos++;
			}
		}

		if (strcmp(commandbuf, "sampling") == 0)
		{
			//"sampling" command
			
			if (strcmp(argumentbuf, "on") == 0)
			{
				Red = 0;
				sampler.attach(&sampletrigger, 0.1f);
				sampleenable = true;
				pc.printf("Sampling enabled\r\n");
			}
			else if (strcmp(argumentbuf, "off") == 0)
			{
				Red = 1;
				sampler.detach();
				sampleenable = false;
				pc.printf("Sampling disabled\r\n");
			}
			else
			{
				pc.printf("Argument is not valid\r\n");
			}
		}
		else if (strcmp(commandbuf, "print") == 0)
		{
			int arg = stringtoint(argumentbuf);
			
			if (arg == -1)
				pc.printf("argument contained non-integer characters\r\n");
			else
				printdata(arg);
		}
		else if (strcmp(commandbuf, "delete") == 0)
		{
			int arg = stringtoint(argumentbuf);
			
			if (arg == -1)
				pc.printf("argument contained non-integer characters\r\n");
			else
				deletedata(arg);
		}
//		else if (strcmp(commandbuf, "dump") == 0)
//		{
//			pc.printf("%i records in memory\r\n", recordcount);
//			for (int i=0; i<SIGBUF_SIZE; i++)
//			{
//				pc.printf("%f\r\n", sigbuffer[i]);
//			}
//		}
		else
		{
			pc.printf("Command was not recognized\r\n");
		}
		
		combufpos = argbufpos = 0;
		memset(commandbuf, 0x0, COMBUF_SIZE);
		memset(argumentbuf, 0x0, ARGBUF_SIZE);
	}
}

int main() {
	//Due to being volatile "sigbuffer" cannot be
	//cleared with memset. We must do it manualy.
	for (int i=0; i<SIGBUF_SIZE; i++)
		sigbuffer[i] = 0.0f;
	
	//attach sampler interupt
	sampler.attach(&sampletrigger, 0.1f);
	
	//start threads
	sample.start(&sampleprocess);
	userinp.start(&userinput);
	
	//pc.scanf("%s", inpbuffer);
	//pc.printf("%s\r\n", inpbuffer);
	 
//	#if MBED_CONF_RTOS_API_PRESENT
//	pc.printf("We have an RTOS\r\n");
//	#else 
//	pc.printf("No RTOS enabled\r\n");
//	#endif

//	while (1) {
//			yellow = !yellow;
//			wait(0.5);
//	}
	
	return 0;
}
