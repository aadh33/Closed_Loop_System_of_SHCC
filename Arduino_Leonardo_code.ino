  // Create JSON object
    StaticJsonDocument<256> doc;
    doc["Sensor"] = "encoder";

    JsonObject encoder1 = doc.createNestedObject("Encoder1Data");
    encoder1["distance"] = currentDistance1;
    encoder1["velocity"] = velocity1;
    encoder1["angle"] = angle1;
    encoder1["angular_velocity"] = angularVelocity1;

    JsonObject encoder2 = doc.createNestedObject("Encoder2Data");
    encoder2["distance"] = currentDistance2;
    encoder2["velocity"] = velocity2;
    encoder2["angle"] = angle2;
    encoder2["angular_velocity"] = angularVelocity2;

    doc["angular_velocity_body"] = angularVelocityBody;
    doc["direction"] = direction;

    // Serialize JSON and send via serial
    String output;
    serializeJson(doc, output);
    mySerial.println(output); //New line for readability
    Serial.println(output);
    // Update previous distance for next velocity calculation
    previousDistance1 = currentDistance1;
    previousDistance2 = currentDistance2;
    previousAngle1 = angle1;
    previousAngle2 = angle2;
    
    // Reset lastTime to the current time to start the next 1-second interval
    lastTime = currentTime;
  }
}



