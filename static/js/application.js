
$(document).ready(function(){
    //connect to the socket server.
    var socket = io.connect('http://' + document.domain + ':' + location.port + '/steps');
    var numbers_received = 0;

    //receive details from server
    socket.on('newnumber', function(msg) {
        console.log("Received number" + msg.number);
        //maintain a list of ten numbers
        // if (numbers_received.length >= 2){
        //     numbers_received.shift()
        // }            
        numbers_received = msg.number;
        // numbers_string = '';
        // for (var i = 0; i < numbers_received.length; i++){
        //     numbers_string = numbers_string + '<p>' + numbers_received[i].toString() + '</p>';
        // }
        $('#steps').html(numbers_received.toString());
    });

});