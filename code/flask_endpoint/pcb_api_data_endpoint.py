from flask import Flask, request, jsonify


params_dict = {"test1":"this is just a test"}
print(f"initial params_dict = {params_dict}")
app = Flask(__name__)

@app.route('/input-data', methods=['GET'])
def input_data():
    # Get the query parameters
    params = request.args
    # Convert the ImmutableMultiDict to a regular dictionary
    temp_dict = params.to_dict()
    for key, value in temp_dict.items():
        params_dict[key] = value
    print(f"params_dict updated to {params_dict}")
    return jsonify(status=200)

@app.route('/get-data', methods=['GET'])
def get_data():
    print(f"get_data gives {params_dict}")
    return jsonify(params_dict=params_dict, status=200, mimetype='application/json')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050)