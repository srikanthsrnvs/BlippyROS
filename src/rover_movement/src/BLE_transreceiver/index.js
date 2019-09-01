var noble = require('noble');
var net = require('net')
var async = require('async')

var motor1Data, motor2Data, motor3Data, motor4Data, motorController;
var client;
const ServiceUUID                    = "9633CB91762444AAAEA2D44A02EBB643"
const motor1Characterisitc           = 'C6BAE32A16FC41378C0E909D473D24A4';
const motor2Characterisitc           = '2E69D35E94CC4B53B610E40AB5223417';
const motor3Characterisitc           = 'FC99D19D4F9E471EBECD41AF9C4C1FA2';
const motor4Characterisitc           = '9E84A0BA69C54EB69438DCF430713EA9';
const motorControllerCharacteristic  = '88BBAA2ADC234331AA02BB3D4DC9D97A';

noble.on('stateChange', state => {
    if (state === 'poweredOn') {
        console.log('Scanning');
        noble.startScanning();
    } else {
        noble.stopScanning();
    }
});

noble.on('discover', peripheral => {
    if (peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids[0] == ServiceUUID.toLowerCase()){
        noble.stopScanning();
        const name = peripheral.advertisement.localName;
        console.log(`Connecting to '${name}' ${peripheral.id}`);
        connectAndSetUp(peripheral);
    }
});

function connectAndSetUp(peripheral) {

  peripheral.connect(error => {
    console.log('Connected to', peripheral.id);

    // specify the services and characteristics to discover
    const serviceUUIDs = [ServiceUUID.toLowerCase()];
    const characteristicUUIDs = [motor1Characterisitc.toLowerCase(), motor2Characterisitc.toLowerCase(), motor3Characterisitc.toLowerCase(), motor4Characterisitc.toLowerCase(), motorControllerCharacteristic.toLowerCase()];

    peripheral.discoverSomeServicesAndCharacteristics(
        serviceUUIDs,
        characteristicUUIDs,
        onServicesAndCharacteristicsDiscovered
    );
  });
  
  peripheral.on('disconnect', () => console.log('disconnected'));
}

function onServicesAndCharacteristicsDiscovered(error, services, characteristics) {
    if(error){
        console.log(error)
    }
    console.log('Discovered services and characteristics');
    motor1Data = characteristics[0];
    motor2Data = characteristics[1];
    motor3Data = characteristics[2];
    motor4Data = characteristics[3];
    motorController = characteristics[4];

    async.parallel([
        function(callback){
            motor1Data.subscribe(error => {
                if (error) {
                    callback(error, null)
                    console.error('Error subscribing to motor1Data: ', error);
                } else {
                    callback(null, null)
                    console.log('Subscribed for motor1Data notifications');
                }
            });
        },
        function(callback){
            motor2Data.subscribe(error => {
                if (error) {
                    callback(error, null)
                    console.error('Error subscribing to motor2Data: ', error);
                } else {
                    callback(null, null)
                    console.log('Subscribed for motor2Data notifications');
                }
            });
        },
        function(callback){
            motor3Data.subscribe(error => {
                if (error) {
                    callback(error, null)
                    console.error('Error subscribing to motor3Data: ', error);
                } else {
                    callback(null, null)
                    console.log('Subscribed for motor3Data notifications');
                }
            });
        },
        function(callback){
            motor4Data.subscribe(error => {
                if (error) {
                    callback(error, null)
                    console.error('Error subscribing to motor4Data: ', error);
                } else {
                    callback(null, null)
                    console.log('Subscribed for motor4Data notifications');
                }
            });
        },
        function(callback){
            motorController.subscribe(error => {
                if (error) {
                    callback(error, null)
                    console.error('Error subscribing to servo: ', error);
                } else {
                    callback(null, null)
                    console.log('Subscribed for servo notifications');
                }
            });
        }
    ], function(err, results){
        if (!err){
            setupSocket()
        }
    })

    motorController.on('data', (data, isNotification) => {
        console.log('Received command from iPad: ' + data);
        client.write(data)
    });
}

function setupSocket(){

    client = new net.Socket();

    client.connect(1337, '127.0.0.1', function() {
        console.log('Connected');
    });

    client.on('data', function(data) {
        console.log('Received data from Rover');
        var buf = JSON.parse(data.toString());

        var motor1JSON = Buffer.from(JSON.stringify(buf[0]));
        var motor2JSON = Buffer.from(JSON.stringify(buf[1]));
        var motor3JSON = Buffer.from(JSON.stringify(buf[2]));
        var motor4JSON = Buffer.from(JSON.stringify(buf[3]));

        console.log("Motor1: ", buf[0])
        console.log("Motor2: ", buf[1])
        console.log("Motor3: ", buf[2])
        console.log("Motor4: ", buf[3])

        motor1Data.write(motor1JSON, true)
        motor2Data.write(motor2JSON, true)
        motor3Data.write(motor3JSON, true)
        motor4Data.write(motor4JSON, true)
    })

    client.on('close', function() {
        console.log('Connection closed');
    });
}