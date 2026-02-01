import OpenSCAD from "./deps/openscad-2026.02.01/openscad.js";

async function load3mf(url) {
    const loader = new Promise(function (resolve, reject) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", url);
        xhr.responseType = "blob";
        xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
            const geometry = xhr.response;
            resolve(geometry);
        } else {
            reject({
            status: xhr.status,
            statusText: xhr.statusText
            });
        }
        };
        xhr.onerror = function () {
        reject({
            status: xhr.status,
            statusText: xhr.statusText
        });
        };
        xhr.send();
    });
    const blob = await loader;
    return blob.arrayBuffer();
}

function prepareCad(cad, body, tools) {
    cad.FS.writeFile("/body.3MF", new Int8Array(body));

    const script = [
        "difference () {",
        '    import("body.3MF");',
    ];

    tools.map((tool, index)=>{
        script.push(`    import("tool${index}.3MF");`);
        cad.FS.writeFile(`/tool${index}.3MF`, new Int8Array(tool));
    });
    script.push('}');

    cad.FS.writeFile("/script.scad", script.join("\n"));
    return cad;
}

async function computeGuitar(msg) {
    try {
        const bodyReqs = msg.data.bodies.map(load3mf);
        const toolReqs = msg.data.tools.map(load3mf);
        const cadReqs = msg.data.bodies.map(() => OpenSCAD({noInitialRun: true}));

        const bodies = await Promise.all(bodyReqs);
        const tools = await Promise.all(toolReqs);
        const cads = await Promise.all(cadReqs);

        const links = [];

        cads.map((cad, index) => {
            prepareCad(cad, bodies[index], tools);
            cad.callMain(["/script.scad", "-o", "/output.stl"]);
            const blob = new Blob([cad.FS.readFile("/output.stl")], { type: "model/stl" });
            links.push(`<a download="part${index + 1}.stl" href="${URL.createObjectURL(blob)}">Download STL for part ${index + 1}</a>`);
        });
        postMessage(links);
    }
    catch(err) {
        postMessage("failed");
        while (Error.isError(err)) {
            if (err.fileName !== undefined && err.fileName !== null && err.lineNumber !== undefined && err.lineNumber !== null) {
                console.log(`${err.name} - ${err.message} at ${err.fileName}:${err.lineNumber}`);
            } else {
                console.log(`${err.name} - ${err.message}`);
            }
            if (err.stack !== undefined && err.stack !== null) {
                console.log("Stack is:\n\n", err.stack);
            }
            err = err.cause;
        }
        if (err !== undefined && err !== null) {
            console.log(err);
        }
    }
}

addEventListener('message', computeGuitar);
