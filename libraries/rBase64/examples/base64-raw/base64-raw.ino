/*

  Raw operation example of rBASE64 Library

  This example shows the calling convention for the various functions.

  For more information about this library please visit us at
  http://github.com/boseji/BASE64

  Created by Abhijit Bose (boseji) on 22/02/16.
  Copyright 2016 - Under creative commons license 3.0:
        Attribution-ShareAlike CC BY-SA

  @version API 1.1.0
  @author boseji - salearj@hotmail.com

*/
#include <rBase64.h>

// Maximum Input we can provide - Can hange based on requirement.
#define BUFFER_MAX 50
// Calculated Fixed output buffer Size
#define OUTPUT_MAX (RBASE64_ENC_SIZECALC(BUFFER_MAX))

// Global Operations Buffer
char buffer[OUTPUT_MAX];
// Output Buffer
char output[OUTPUT_MAX];
// Input for Encoding
char input[BUFFER_MAX] = "Hello There";
// Used to store the Length of the Generated output
size_t len;

void setup() {
  Serial.begin(115200);
}

void loop() {
	delay(5000);
	strcpy(buffer, input);
	Serial.print(" Base64 Encoding: ");
	Serial.println(input);
	if (rbase64_enc_len(strlen(buffer)) <= OUTPUT_MAX) {
		Serial.println("  - We have Sufficient Length of Output Buffer");
		len = rbase64_encode(output, buffer, strlen(buffer));
		Serial.print("  - Encoded Output Buffer Length: ");
		Serial.println(len);
		Serial.print("  - Encoded: ");
		Serial.println(output);
	} else {
		Serial.println("  - We do not have sufficient Length of Output Buffer");
		return;
	}
	Serial.println();
	strcpy(buffer, output);
	Serial.print(" Base64 Decoding: ");
	Serial.println(buffer);	
	if (rbase64_dec_len(buffer, strlen(buffer)) <= OUTPUT_MAX) {
		Serial.println("  - We have Sufficient Length of Output Buffer");
		len = rbase64_decode(output, buffer, strlen(buffer));
		Serial.print("  - Decoded Output Buffer Length: ");
		Serial.println(len);
		Serial.print("  - Decoded: ");
		Serial.println(output);
	} else {
		Serial.println("  - We do not have sufficient Length of Output Buffer");
		return;
	}
	Serial.println();
}
