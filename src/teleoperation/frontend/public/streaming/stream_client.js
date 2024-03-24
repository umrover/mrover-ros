// Requires a canvas named "stream-{number}" to be present in the DOM
// The port of the websocket is 8080 + number
function startStream(number) {
    // Corresponds to H.265
    // I can't figure out what the other values are for... obtained via guess and check
    const STREAM_CODEC = 'hvc1.1.2.L90.90';
    const STREAM_WIDTH = 640;
    const STREAM_HEIGHT = 480;

    const vertexShaderSource = `
attribute vec2 xy;

varying highp vec2 uv;

void main(void) {
    gl_Position = vec4(xy, 0.0, 1.0);
    // Map vertex coordinates (-1 to +1) to UV coordinates (0 to 1).
    // UV coordinates are Y-flipped relative to vertex coordinates.
    uv = vec2((1.0 + xy.x) / 2.0, (1.0 - xy.y) / 2.0);
}
`;

    const fragmentShaderSource = `
varying highp vec2 uv;

uniform sampler2D texture;

void main(void) {
    gl_FragColor = texture2D(texture, uv);
}
`;

    const canvas = document.getElementById(`stream-${number}`);
    if (!canvas) throw new Error(`Canvas with id "stream-${number}" not found`);

    // This WebGL stuff seems like a lot, but it's just setting up a shader that can render a texture
    // This texture is uploaded to whenever we get a frame from the decoder
    // More complex than just using a 2D canvas but *may* have lower latency

    const gl = canvas.getContext('webgl2');

    const vertexShader = gl.createShader(gl.VERTEX_SHADER);
    gl.shaderSource(vertexShader, vertexShaderSource);
    gl.compileShader(vertexShader);
    if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
        throw gl.getShaderInfoLog(vertexShader);
    }

    const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
    gl.shaderSource(fragmentShader, fragmentShaderSource);
    gl.compileShader(fragmentShader);
    if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
        throw gl.getShaderInfoLog(fragmentShader);
    }

    const shaderProgram = gl.createProgram();
    gl.attachShader(shaderProgram, vertexShader);
    gl.attachShader(shaderProgram, fragmentShader);
    gl.linkProgram(shaderProgram);
    if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
        throw gl.getProgramInfoLog(shaderProgram);
    }
    gl.useProgram(shaderProgram);

    // Vertex coordinates, clockwise from bottom-left
    const vertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        -1.0, -1.0,
        -1.0, +1.0,
        +1.0, +1.0,
        +1.0, -1.0,
    ]), gl.STATIC_DRAW);

    const xyLocation = gl.getAttribLocation(shaderProgram, "xy");
    gl.vertexAttribPointer(xyLocation, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(xyLocation);

    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

    const decoder = new VideoDecoder({
        output(frame) {
            canvas.width = frame.displayWidth;
            canvas.height = frame.displayHeight;
            // Upload the frame to the texture
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, frame);
            // Close immediately to free up resources
            // Otherwise the stream will halt
            frame.close();

            gl.viewport(0, 0, canvas.width, canvas.height);
            gl.clearColor(0, 0, 0, 1);
            gl.clear(gl.COLOR_BUFFER_BIT);

            gl.drawArrays(gl.TRIANGLE_FAN, 0, 4);
        },
        error(e) {
            throw e;
        }
    });
    // TODO(quintin): Need to know the size ahead of time. Perhaps put in a packet
    decoder.configure({
        codec: STREAM_CODEC,
        codedWidth: STREAM_WIDTH,
        codedHeight: STREAM_HEIGHT,
    })

    // TODO(quintin): Set IP too
    const ws = new WebSocket(`ws://localhost:808${number}`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
        console.log('Connected to server');
    };
    ws.onclose = () => {
        console.log('Disconnected from server');
        decoder.close();
        // This recursive-ness stops after the canvas element is removed
        setTimeout(() => startStream(number), 3000);
    };
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        ws.close()
    };
    ws.onmessage = (event) => {
        // If the canvas disappears this means the frontend removed it,
        // and we should stop the stream (and all following retries)
        if (!document.getElementById(`stream-${number}`)) {
            // The canvas has been removed, so stop decoding
            ws.close();
            return;
        }
        // TODO(quintin): Should the values besides "data" be set better? Parsed from the packet?
        decoder.decode(new EncodedVideoChunk({
            type: "key",
            timestamp: performance.now(),
            duration: 1000 / 30,
            data: event.data,
        }));
    };
}